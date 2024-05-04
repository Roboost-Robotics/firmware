/**
 * @file core.cpp
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief This file contains the main functionality for controlling the Roboost
 * robot connected over serial to the main computer.
 * @version 1.2
 * @date 2024-04-30
 *
 * @copyright Copyright (c) 2023
 *
 */

// TODO: Use const types
// TODO: Instead of Serial.println, use different approach for logging

#include <Arduino.h>
#include <ArduinoEigen.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <roboost/utils/rcl_checks.h>

#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>

#include <roboost/utils/timing.hpp>

#include "conf_hardware.h"
#include "conf_network.h"
#include <roboost/motor_control/encoder.hpp>
#include <roboost/motor_control/motor_drivers/l298n_motor_driver.hpp>
#include <roboost/motor_control/pid_motor_controller.hpp>
#include <roboost/motor_control/robot_controller.hpp>
#include <roboost/motor_control/simple_motor_controller.hpp>
#include <roboost/utils/logging.hpp>

#define MOTOR_COUNT 4

roboost::motor_control::L298NMotorDriver drivers[MOTOR_COUNT] = {
    {M0_IN1, M0_IN2, M0_ENA, M0_PWM_CNL}, {M1_IN1, M1_IN2, M1_ENA, M1_PWM_CNL}, {M2_IN1, M2_IN2, M2_ENA, M2_PWM_CNL}, {M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL}};

// HalfQuadEncoder encoders[MOTOR_COUNT] = {
//     {M0_ENC_A, M0_ENC_B, M0_ENC_RESOLUTION},
//     {M1_ENC_A, M1_ENC_B, M1_ENC_RESOLUTION},
//     {M2_ENC_A, M2_ENC_B, M2_ENC_RESOLUTION},
//     {M3_ENC_A, M3_ENC_B, M3_ENC_RESOLUTION}};

roboost::motor_control::DummyEncoder encoders[MOTOR_COUNT] = {{M0_ENC_RESOLUTION}, {M1_ENC_RESOLUTION}, {M2_ENC_RESOLUTION}, {M3_ENC_RESOLUTION}};

constexpr double base_kp = 0.105;
constexpr double base_ki = 0.125;
constexpr double modifier_ki_linear = 2.0;
constexpr double modifier_ki_rotational = 1.1;
constexpr double base_kd = 0.005;
constexpr double max_expected_sampling_time = 0.2;
constexpr double max_integral = 5.2;

roboost::controllers::PIDController controllers[MOTOR_COUNT] = {{base_kp, base_ki, base_kd, max_expected_sampling_time, max_integral},
                                                                {base_kp, base_ki, base_kd, max_expected_sampling_time, max_integral},
                                                                {base_kp, base_ki, base_kd, max_expected_sampling_time, max_integral},
                                                                {base_kp, base_ki, base_kd, max_expected_sampling_time, max_integral}};

roboost::filters::NoFilter encoder_input_filters[MOTOR_COUNT] = {{}, {}, {}, {}};

roboost::filters::NoFilter motor_output_filters[MOTOR_COUNT] = {{}, {}, {}, {}};

static double MIN_OUTPUT = 0.35;

roboost::motor_control::PIDMotorController motor_controllers[MOTOR_COUNT] = {{drivers[0], encoders[0], controllers[0], encoder_input_filters[0], motor_output_filters[0], MIN_OUTPUT},
                                                                             {drivers[1], encoders[1], controllers[1], encoder_input_filters[1], motor_output_filters[1], MIN_OUTPUT},
                                                                             {drivers[2], encoders[2], controllers[2], encoder_input_filters[2], motor_output_filters[2], MIN_OUTPUT},
                                                                             {drivers[3], encoders[3], controllers[3], encoder_input_filters[3], motor_output_filters[3], MIN_OUTPUT}};

roboost::motor_control::MotorControllerManager motor_control_manager{&motor_controllers[0], &motor_controllers[1], &motor_controllers[2], &motor_controllers[3]};

roboost::kinematics::MecanumKinematics4W kinematics(WHEEL_RADIUS, WHEEL_BASE, TRACK_WIDTH);
roboost::robot_controller::RobotVelocityController robot_controller(motor_control_manager, &kinematics);

roboost::filters::MovingAverageFilter cmd_vel_filter_x = roboost::filters::MovingAverageFilter(2);
roboost::filters::MovingAverageFilter cmd_vel_filter_y = roboost::filters::MovingAverageFilter(2);
roboost::filters::MovingAverageFilter cmd_vel_filter_theta = roboost::filters::MovingAverageFilter(4);

roboost::logging::SerialLogger& logger = roboost::logging::SerialLogger::getInstance(Serial);

Eigen::Matrix<double, 3, 1> smoothed_cmd_vel;

rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t odom_publisher;
rcl_publisher_t joint_state_publisher, wanted_joint_state_publisher;

rcl_timer_t publish_timer, sync_timer;

geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__JointState joint_state_msg, wanted_joint_state_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

unsigned long last_time = 0;
Eigen::Vector3d pose = Eigen::Vector3d::Zero();

unsigned long last_time_sync_ms = 0;
unsigned long last_time_sync_ns = 0;
const unsigned long time_sync_interval = 1000;
const int sync_timeout_ms = 500;
int64_t synced_time_ms = 0;
int64_t synced_time_ns = 0;

roboost::timing::TimingService& timing_service = roboost::timing::TimingService::get_instance();

IntervalTimer control_loop_timer;

// Predefined global or static data
static const char* odom_frame_id = "odom";
static const char* base_link_frame_id = "base_link";
static const char* joint_state_frame_id = "base_link";
static const char* joint_names[] = {"wheel_front_left_joint", "wheel_front_right_joint", "wheel_back_left_joint", "wheel_back_right_joint"};

// clang-format off
static const double default_covariance[36] = {
    0.8, 0, 0, 0, 0, 0, 0,
    0, 0.8, 0, 0, 0, 0, 0,
    0, 0, 0.8, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0.8
};
// clang-format on

// Function prototypes
void cmd_vel_subscription_callback(const void* msgin);
void set_ros_timestamp(std_msgs__msg__Header& header, unsigned long sync_ms, unsigned long sync_ns);
void publish_joint_states(const Eigen::Vector4d& velocities, double dt);
void update_odometry(const Eigen::Vector3d& velocity, double dt);
void pub_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void init_odometry_msg();
void init_joint_state_msg();
void init_wanted_joint_state_msg();
void init_microros();
void sync_callback();

/**
 * @brief Setup function for initializing micro-ROS, pin modes, etc.
 *
 */
void setup()
{
    // Configure serial transport
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    // Setup Timingservice
    // timing_service.reset();

    set_microros_serial_transports(Serial);

    init_microros();

    digitalWrite(LED_BUILTIN, HIGH);

    // timing_service.addTask([]() { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); }, TIMING_MS_TO_US(500), TIMING_MS_TO_US(1000), "LED blink");

    // timing_service.addTask([]() { robot_controller.update(); }, TIMING_MS_TO_US(20), TIMING_MS_TO_US(50),
    //                        "Contoller update"); // Update robot controller every
    //                                             // 20ms with a timeout of 50ms

    // timing_service.addTask([]() { RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); }, TIMING_MS_TO_US(200), TIMING_MS_TO_US(500), "Executor spin");

    // timing_service.addTask([]() { pub_callback(); }, TIMING_MS_TO_US(100), TIMING_MS_TO_US(200), "Publisher");

    // timing_service.addTask([]() { sync_callback(); }, TIMING_MS_TO_US(1000), TIMING_MS_TO_US(2000), "Time sync");
    // timing_service.addTask(
    //     []()
    //     {
    //         Serial.print("vx: ");
    //         Serial.print(robot_controller.get_robot_velocity()(0));
    //         Serial.print(" vy: ");
    //         Serial.print(robot_controller.get_robot_velocity()(1));
    //         Serial.print(" vtheta: ");
    //         Serial.print(robot_controller.get_robot_velocity()(2));
    //         Serial.print(" dt: ");
    //         Serial.print(timing_service.getDeltaTime());
    //         Serial.print("us");

    //         Serial.print(" wanted:: vx: ");
    //         Serial.print(robot_controller.get_set_wheel_velocities()(0));
    //         Serial.print(" vy: ");
    //         Serial.print(robot_controller.get_set_wheel_velocities()(1));
    //         Serial.print(" vtheta: ");
    //         Serial.println(robot_controller.get_set_wheel_velocities()(2));
    //     },
    //     TIMING_MS_TO_US(1000), TIMING_MS_TO_US(2000), "Robot state");

    // Hardware timer for teensy 4.0 controller update
}

/**
 * @brief Main loop for continuously updating and publishing the robot's
 * odometry.
 *
 */
void loop()
{
    // timing_service.update();
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)), logger);
}

void print_free_heap()
{
#ifdef ESP32
    Serial.print("free heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" | min free heap: ");
    Serial.print(ESP.getMinFreeHeap());
    Serial.print(" | diff: ");
    Serial.println(ESP.getFreeHeap() - ESP.getMinFreeHeap());
#endif
}

void init_microros()
{
    // IPAddress agent_ip(AGENT_IP);
    // uint16_t agent_port = AGENT_PORT;
    // Serial.println("Initializing micro-ROS transport...");
    // print_free_heap();
    // set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip,
    //                              agent_port);

    allocator = rcl_get_default_allocator();

    INIT(rclc_support_init(&support, 0, NULL, &allocator), logger);
    INIT(rclc_node_init_default(&node, "roboost_pmc_node", "", &support), logger);

    INIT(rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"), logger);
    INIT(rclc_publisher_init_default(&joint_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states"), logger);
    // INIT(rclc_publisher_init_default(&wanted_joint_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "wanted_joint_states"));

    INIT(rclc_timer_init_default(&publish_timer, &support, RCL_MS_TO_NS(100), pub_timer_callback), logger);
    INIT(rclc_timer_init_default(&sync_timer, &support, RCL_MS_TO_NS(time_sync_interval), sync_timer_callback), logger);

    INIT(rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"), logger);

    INIT(rclc_executor_init(&executor, &support.context, 3, &allocator), logger);
    INIT(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &cmd_vel_subscription_callback, ON_NEW_DATA), logger);
    INIT(rclc_executor_add_timer(&executor, &publish_timer), logger);

    // Synchronize time with the agent
    rmw_uros_sync_session(sync_timeout_ms);

    if (rmw_uros_epoch_synchronized())
    {
        // Get time in milliseconds or nanoseconds
        synced_time_ms = rmw_uros_epoch_millis();
        synced_time_ns = rmw_uros_epoch_nanos();
    }
    else
    {
        Serial.println("Error in sync_timer_callback: time not synchronized\n");
    }

    // Delay and blink once
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    init_odometry_msg();
    init_joint_state_msg();
    // init_wanted_joint_state_msg();
}

/**
 * @brief Initialize the odometry message.
 *
 */
void init_odometry_msg()
{
    odom_msg.header.frame_id.data = const_cast<char*>(odom_frame_id);
    odom_msg.header.frame_id.size = strlen(odom_frame_id);
    odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
    odom_msg.child_frame_id.data = const_cast<char*>(base_link_frame_id);
    odom_msg.child_frame_id.size = strlen(base_link_frame_id);
    odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;

    memcpy(odom_msg.pose.covariance, default_covariance, sizeof(default_covariance));
    memcpy(odom_msg.twist.covariance, default_covariance, sizeof(default_covariance));

    // Add sample data
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
}

/**
 * @brief Initialize the joint state message.
 *
 */
void init_joint_state_msg()
{
    joint_state_msg.header.frame_id.data = "base_link";
    rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 4);
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "wheel_front_left_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "wheel_front_right_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[2], "wheel_back_left_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[3], "wheel_back_right_joint");
    joint_state_msg.name.size = 4;
    joint_state_msg.name.capacity = 4;

    joint_state_msg.position.data = (double*)malloc(4 * sizeof(double));
    joint_state_msg.position.size = 4;
    joint_state_msg.position.capacity = 4;

    joint_state_msg.velocity.data = (double*)malloc(4 * sizeof(double));
    joint_state_msg.velocity.size = 4;
    joint_state_msg.velocity.capacity = 4;

    // Add sample data
    joint_state_msg.position.data[0] = 0.0;
    joint_state_msg.position.data[1] = 0.0;
    joint_state_msg.position.data[2] = 0.0;
    joint_state_msg.position.data[3] = 0.0;
    joint_state_msg.velocity.data[0] = 0.0;
    joint_state_msg.velocity.data[1] = 0.0;
    joint_state_msg.velocity.data[2] = 0.0;
    joint_state_msg.velocity.data[3] = 0.0;

    // joint_state_msg.header.frame_id.data = const_cast<char*>(joint_state_frame_id);
    // joint_state_msg.name.size = 4;
    // joint_state_msg.name.capacity = 4;

    // for (int i = 0; i < 4; i++)
    // {
    //     rosidl_runtime_c__String__assign(&joint_state_msg.name.data[i], joint_names[i]);
    // }

    // static double position[4] = {0.0, 0.0, 0.0, 0.0};
    // static double velocity[4] = {0.0, 0.0, 0.0, 0.0};

    // joint_state_msg.position.data = position;
    // joint_state_msg.position.size = 4;
    // joint_state_msg.position.capacity = 4;

    // joint_state_msg.velocity.data = velocity;
    // joint_state_msg.velocity.size = 4;
    // joint_state_msg.velocity.capacity = 4;
}

/**
 * @brief Initialize the wanted joint state message.
 *
 */
void init_wanted_joint_state_msg()
{
    wanted_joint_state_msg.header.frame_id.data = const_cast<char*>(joint_state_frame_id);
    wanted_joint_state_msg.name.size = 4;
    wanted_joint_state_msg.name.capacity = 4;

    for (int i = 0; i < 4; i++)
    {
        rosidl_runtime_c__String__assign(&wanted_joint_state_msg.name.data[i], joint_names[i]);
    }

    static double position[4] = {0.0, 0.0, 0.0, 0.0};

    wanted_joint_state_msg.position.data = position;
    wanted_joint_state_msg.position.size = 4;
    wanted_joint_state_msg.position.capacity = 4;

    // Add sample data
    wanted_joint_state_msg.position.data[0] = 0.0;
    wanted_joint_state_msg.position.data[1] = 0.0;
    wanted_joint_state_msg.position.data[2] = 0.0;
    wanted_joint_state_msg.position.data[3] = 0.0;
}

/**
 * @brief Callback function for handling incoming cmd_vel (velocity command)
 * messages.
 *
 * @param msgin Pointer to the received geometry_msgs__msg__Twist message.
 *
 * @note The Twist message has following structure:
 *
 * std_msgs/Header header
 * geometry_msgs/Vector3 linear
 * geometry_msgs/Vector3 angular
 *
 */
void cmd_vel_subscription_callback(const void* msgin)
{
    // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    const auto* msg = reinterpret_cast<const geometry_msgs__msg__Twist*>(msgin);

    smoothed_cmd_vel(0) = cmd_vel_filter_x.update(msg->linear.x);
    smoothed_cmd_vel(1) = cmd_vel_filter_y.update(msg->linear.y);
    smoothed_cmd_vel(2) = cmd_vel_filter_theta.update(msg->angular.z);

    // Based on max velocity multiply the ki value by a factor
    if (abs(smoothed_cmd_vel(0)) > 0.5 || abs(smoothed_cmd_vel(1)) > 0.5)
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            controllers[i].set_ki(base_ki * modifier_ki_linear);
        }
    }
    else if (abs(smoothed_cmd_vel(2)) > 1.0)
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            controllers[i].set_ki(base_ki * modifier_ki_rotational);
        }
    }
    else
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            controllers[i].set_ki(base_ki);
        }
    }

    robot_controller.set_latest_command(smoothed_cmd_vel);
}

/**
 * @brief Helper function to set the ROS timestamp for a message.
 *
 * @param header Header of the message
 * @param sync_ms Synced time in milliseconds
 * @param sync_ns Nano second part of the synced time
 */
void set_ros_timestamp(std_msgs__msg__Header& header, unsigned long sync_ms, unsigned long sync_ns)
{
    header.stamp.sec = (sync_ms + millis() - last_time_sync_ms) / 1000;
    header.stamp.nanosec = sync_ns + (micros() * 1000 - last_time_sync_ns) % 1000000000;
}

/**
 * @brief Helper function to publish joint states.
 *
 * @param velocities Vector of wheel velocities
 * @param dt Time step
 */
void publish_joint_states(const Eigen::Vector4d& velocities, double dt)
{
    for (int i = 0; i < 4; i++)
    {
        joint_state_msg.position.data[i] += velocities(i) * dt;
        joint_state_msg.velocity.data[i] = velocities(i);
    }
    set_ros_timestamp(joint_state_msg.header, synced_time_ms, synced_time_ns);
    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL), logger);
}

/**
 * @brief Helper function to publish desired joint states.
 *
 * @param velocities Vector of wheel velocities
 * @param dt Time step
 */
void publish_wanted_joint_states(const Eigen::Vector4d& velocities, double dt)
{
    for (int i = 0; i < 4; i++)
    {
        wanted_joint_state_msg.position.data[i] = velocities(i);
    }
    set_ros_timestamp(wanted_joint_state_msg.header, synced_time_ms, synced_time_ns);
    RCSOFTCHECK(rcl_publish(&wanted_joint_state_publisher, &wanted_joint_state_msg, NULL), logger);
}

/**
 * @brief Helper function to update odometry.
 *
 * @param velocity Robot velocity
 * @param dt Time step
 */
void update_odometry(const Eigen::Vector3d& velocity, double dt)
{
    pose(0) += velocity(0) * cos(pose(2)) * dt - velocity(1) * sin(pose(2)) * dt;
    pose(1) += velocity(0) * sin(pose(2)) * dt + velocity(1) * cos(pose(2)) * dt;
    pose(2) += velocity(2) * dt;
    pose(2) = atan2(sin(pose(2)), cos(pose(2)));

    odom_msg.pose.pose.position.x = pose(0);
    odom_msg.pose.pose.position.y = pose(1);
    odom_msg.pose.pose.orientation.w = cos(pose(2) / 2.0);
    odom_msg.pose.pose.orientation.z = sin(pose(2) / 2.0);
    odom_msg.twist.twist.linear.x = velocity(0);
    odom_msg.twist.twist.linear.y = velocity(1);
    odom_msg.twist.twist.angular.z = velocity(2);
}

/**
 * @brief Callback function for the publish timer.
 *
 * @param timer Timer object
 * @param last_call_time Last call time
 */
void pub_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    if (timer == NULL)
    {
        Serial.println("Error in timer_callback: timer parameter is NULL\n");
        return;
    }

    // double dt = MICROS_TO_SECONDS_DOUBLE(timing_service.getDeltaTime());
    // Eigen::Vector3d robot_velocity = robot_controller.get_robot_velocity();

    // Update odometry
    // update_odometry(robot_velocity, dt);
    set_ros_timestamp(odom_msg.header, synced_time_ms, synced_time_ns);
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL), logger);

    // TODO: Remove this
    set_ros_timestamp(joint_state_msg.header, synced_time_ms, synced_time_ns);
    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL), logger);

    // // Publish joint states
    // Eigen::Vector4d wheel_velocities = kinematics.calculate_wheel_velocity(robot_velocity);
    // publish_joint_states(wheel_velocities, dt);

    // // Publish desired joint states
    // Eigen::Vector4d wanted_wheel_velocities = robot_controller.get_set_wheel_velocities();
    // publish_wanted_joint_states(wanted_wheel_velocities, dt);
}

/**
 * @brief Callback function for the sync timer.
 *
 * @param timer Timer object
 * @param last_call_time Last call time
 */
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    if (timer == NULL)
    {
        Serial.println("Error in timer_callback: timer parameter is NULL\n");
        return;
    }
    // Synchronize time with the agent
    rmw_uros_sync_session(sync_timeout_ms);

    if (rmw_uros_epoch_synchronized())
    {
        // Get time in milliseconds or nanoseconds
        synced_time_ms = rmw_uros_epoch_millis();
        synced_time_ns = rmw_uros_epoch_nanos();
        last_time_sync_ms = millis();
        last_time_sync_ns = micros() * 1000;
    }
    else
    {
        Serial.println("Error in sync_timer_callback: time not synchronized\n");
    }
}

void sync_callback()
{
    // Synchronize time with the agent
    rmw_uros_sync_session(sync_timeout_ms);

    if (rmw_uros_epoch_synchronized())
    {
        // Get time in milliseconds or nanoseconds
        synced_time_ms = rmw_uros_epoch_millis();
        synced_time_ns = rmw_uros_epoch_nanos();
    }
    else
    {
        Serial.println("Error in sync_timer_callback: time not synchronized\n");
    }
}