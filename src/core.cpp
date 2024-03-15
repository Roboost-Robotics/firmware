/**
 * @file core.cpp
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief This file contains the main functionality for controlling a mecanum
 * robot using micro-ROS via Serial.
 * @version 1.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 * @todo Refactor the code to use ROS data types instead of Eigen.
 * @todo Add joint state controller (similar to velocity controller).
 *
 */

#define DEBUG
#define DEBUG_TIME

#include <Arduino.h>
#include <ArduinoEigen.h>
#include <micro_ros_platformio.h>

#include "rcl_checks.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>

#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/int64.h>

#include "conf_hardware.h"
// #include "conf_network.h"
#include "motor-control/encoder.hpp"
#include "motor-control/motor-drivers/l298n_motor_driver.hpp"
#include "motor-control/pid_motor_controller.hpp"
#include "motor-control/simple_motor_controller.hpp"
#include "velocity_controller.hpp"

#include "publishers/odometry_publisher.hpp"

L298NMotorDriver driver_M0(M0_IN1, M0_IN2, M0_ENA, M0_PWM_CNL);
L298NMotorDriver driver_M1(M1_IN1, M1_IN2, M1_ENA, M1_PWM_CNL);
L298NMotorDriver driver_M2(M2_IN1, M2_IN2, M2_ENA, M2_PWM_CNL);
L298NMotorDriver driver_M3(M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL);

HalfQuadEncoder encoder_M0(M0_ENC_A, M0_ENC_B, M0_ENC_RESOLUTION);
HalfQuadEncoder encoder_M1(M1_ENC_A, M1_ENC_B, M1_ENC_RESOLUTION);
HalfQuadEncoder encoder_M2(M2_ENC_A, M2_ENC_B, M2_ENC_RESOLUTION);
HalfQuadEncoder encoder_M3(M3_ENC_A, M3_ENC_B, M3_ENC_RESOLUTION);

double base_kp = 0.105;
// double modifier_kp = 1.0;
double base_ki = 0.125;
double modifier_ki_linear = 2.0;
double modifier_ki_rotational = 1.1;
double base_kd = 0.005;
// double modifier_kd = 1.0;
double max_expected_sampling_time = 0.2;
double max_integral = 5.2;

PIDController controller_M0(base_kp, base_ki, base_kd,
                            max_expected_sampling_time, max_integral);
PIDController controller_M1(base_kp, base_ki, base_kd,
                            max_expected_sampling_time, max_integral);
PIDController controller_M2(base_kp, base_ki, base_kd,
                            max_expected_sampling_time, max_integral);
PIDController controller_M3(base_kp, base_ki, base_kd,
                            max_expected_sampling_time, max_integral);

NoFilter encoder_input_filter_M0 = NoFilter();
NoFilter encoder_input_filter_M1 = NoFilter();
NoFilter encoder_input_filter_M2 = NoFilter();
NoFilter encoder_input_filter_M3 = NoFilter();

NoFilter motor_output_filter_M0 = NoFilter();
NoFilter motor_output_filter_M1 = NoFilter();
NoFilter motor_output_filter_M2 = NoFilter();
NoFilter motor_output_filter_M3 = NoFilter();

static double MIN_OUTPUT = 0.35;

PIDMotorController motor_controller_M0(driver_M0, encoder_M0, controller_M0,
                                       encoder_input_filter_M0,
                                       motor_output_filter_M0, MIN_OUTPUT);
PIDMotorController motor_controller_M1(driver_M1, encoder_M1, controller_M1,
                                       encoder_input_filter_M1,
                                       motor_output_filter_M1, MIN_OUTPUT);
PIDMotorController motor_controller_M2(driver_M2, encoder_M2, controller_M2,
                                       encoder_input_filter_M2,
                                       motor_output_filter_M2, MIN_OUTPUT);
PIDMotorController motor_controller_M3(driver_M3, encoder_M3, controller_M3,
                                       encoder_input_filter_M3,
                                       motor_output_filter_M3, MIN_OUTPUT);

MotorControllerManager motor_control_manager{
    {&motor_controller_M0, &motor_controller_M1, &motor_controller_M2,
     &motor_controller_M3}};

MecanumKinematics4W kinematics(WHEEL_RADIUS, WHEEL_BASE, TRACK_WIDTH);
VelocityController robot_controller(motor_control_manager, &kinematics);

MovingAverageFilter cmd_vel_filter_x = MovingAverageFilter(2);
MovingAverageFilter cmd_vel_filter_y = MovingAverageFilter(2);
MovingAverageFilter cmd_vel_filter_rot = MovingAverageFilter(4);

Eigen::Matrix<double, 3, 1> smoothed_cmd_vel;

rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t joint_state_publisher, wanted_joint_state_publisher;
rcl_publisher_t delta_time_publisher;

OdometryPublisher odom_publisher;

geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__JointState joint_state_msg, wanted_joint_state_msg;
// std_msgs__msg__Float32 delta_time_msg;
std_msgs__msg__Int64 delta_time_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

unsigned long last_time = 0;
Eigen::Vector3d pose = Eigen::Vector3d::Zero();

unsigned long last_time_sync_ms = 0;
const unsigned long time_sync_interval = 1000;
const int timeout_ms = 500;
int64_t synced_time_ms = 0;
int64_t synced_time_ns = 0;

rcl_time_point_value_t global_current_time = 0;

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
    const auto* msg = reinterpret_cast<const geometry_msgs__msg__Twist*>(msgin);

    smoothed_cmd_vel(0) = cmd_vel_filter_x.update(msg->linear.x);
    smoothed_cmd_vel(1) = cmd_vel_filter_y.update(msg->linear.y);
    smoothed_cmd_vel(2) = cmd_vel_filter_rot.update(msg->angular.z);

    // Based on max velocity multiply the ki value by a factor
    if (abs(smoothed_cmd_vel(0)) > 0.5 || abs(smoothed_cmd_vel(1)) > 0.5)
    {
        controller_M0.set_ki(base_ki * modifier_ki_linear);
        controller_M1.set_ki(base_ki * modifier_ki_linear);
        controller_M2.set_ki(base_ki * modifier_ki_linear);
        controller_M3.set_ki(base_ki * modifier_ki_linear);
    }
    else if (abs(smoothed_cmd_vel(2)) > 1.0)
    {
        controller_M0.set_ki(base_ki * modifier_ki_rotational);
        controller_M1.set_ki(base_ki * modifier_ki_rotational);
        controller_M2.set_ki(base_ki * modifier_ki_rotational);
        controller_M3.set_ki(base_ki * modifier_ki_rotational);
    }
    else
    {
        controller_M0.set_ki(base_ki);
        controller_M1.set_ki(base_ki);
        controller_M2.set_ki(base_ki);
        controller_M3.set_ki(base_ki);
    }

    robot_controller.set_latest_command(smoothed_cmd_vel);
}

/**
 * @brief Setup function for initializing micro-ROS, pin modes, etc.
 *
 */
void setup()
{
    Serial.begin(115200);

    set_microros_serial_transports(Serial);

    delay(1000);
    pinMode(LED_BUILTIN, OUTPUT);

    allocator = rcl_get_default_allocator();

    // create init_options
    // clang-format off
    RCCHECK(rclc_support_init(
        &support,
        0,
        NULL,
        &allocator));
    RCCHECK(rclc_node_init_default(
        &node,
        "roboost_pmc_node",
        "",
        &support));
    odom_publisher.initialize(&node, "odom", "base_link", "odom");
    RCCHECK(rclc_publisher_init_default(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states"));
    RCCHECK(rclc_publisher_init_default(
        &wanted_joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "wanted_joint_states"));
    RCCHECK(rclc_publisher_init_default(
        &delta_time_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
        "delta_time/PMC"));
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    RCCHECK(rclc_executor_init(
        &executor,
        &support.context,
        1,
        &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_vel_subscriber,
        &twist_msg,
        &cmd_vel_subscription_callback,
        ON_NEW_DATA));
    // clang-format on

    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);

    char* base_link_name = strdup("base_link");

    // Initialize the joint state message
    joint_state_msg.header.frame_id.data = base_link_name;
    rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 4);
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0],
                                     "wheel_front_left_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1],
                                     "wheel_front_right_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[2],
                                     "wheel_back_left_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[3],
                                     "wheel_back_right_joint");
    joint_state_msg.name.size = 4;
    joint_state_msg.name.capacity = 4;

    joint_state_msg.position.data = (double*)malloc(4 * sizeof(double));
    joint_state_msg.position.size = 4;
    joint_state_msg.position.capacity = 4;

    joint_state_msg.velocity.data = (double*)malloc(4 * sizeof(double));
    joint_state_msg.velocity.size = 4;
    joint_state_msg.velocity.capacity = 4;

    // Initialize the wanted joint state message
    wanted_joint_state_msg.header.frame_id.data = base_link_name;
    rosidl_runtime_c__String__Sequence__init(&wanted_joint_state_msg.name, 4);
    rosidl_runtime_c__String__assign(&wanted_joint_state_msg.name.data[0],
                                     "wheel_front_left_joint");
    rosidl_runtime_c__String__assign(&wanted_joint_state_msg.name.data[1],
                                     "wheel_front_right_joint");
    rosidl_runtime_c__String__assign(&wanted_joint_state_msg.name.data[2],
                                     "wheel_back_left_joint");
    rosidl_runtime_c__String__assign(&wanted_joint_state_msg.name.data[3],
                                     "wheel_back_right_joint");
    wanted_joint_state_msg.name.size = 4;
    wanted_joint_state_msg.name.capacity = 4;

    wanted_joint_state_msg.velocity.data = (double*)malloc(4 * sizeof(double));
    wanted_joint_state_msg.velocity.size = 4;
    wanted_joint_state_msg.velocity.capacity = 4;

    free(base_link_name);
}

/**
 * @brief Main loop for continuously updating and publishing the robot's
 * odometry.
 *
 */
void loop()
{
    // Calculate the delta time
    unsigned long now = millis();
    double dt = (now - last_time) / 1000.0;
    last_time = now;

    // // Publish the delta time
    // delta_time_msg.data = dt;
    // RCSOFTCHECK(rcl_publish(&delta_time_publisher, &delta_time_msg, NULL));

    // Time synchronization
    if (millis() - last_time_sync_ms > time_sync_interval)
    {
        // Synchronize time with the agent
        rmw_uros_sync_session(timeout_ms);

        if (rmw_uros_epoch_synchronized())
        {
            // Get time in milliseconds or nanoseconds
            synced_time_ns = rmw_uros_epoch_nanos();  // Contains the epoch time
                                                      // in nanoseconds
            synced_time_ms = rmw_uros_epoch_millis(); // Contains the epoch time
                                                      // in milliseconds
            last_time_sync_ms = millis();

            // Set the global current time
            global_current_time = synced_time_ns;

            // publish global_current_time time on delta_time topic
            delta_time_msg.data = global_current_time;
            RCSOFTCHECK(
                rcl_publish(&delta_time_publisher, &delta_time_msg, NULL));
        }
    }

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    robot_controller.update();

    Eigen::Vector3d robot_velocity = robot_controller.get_robot_velocity();

    odom_publisher.update(robot_velocity);
    odom_publisher.publish();

    // Update the joint state message

    Eigen::Vector4d wheel_velocities =
        kinematics.calculate_wheel_velocity(robot_velocity);

    joint_state_msg.position.data[0] += wheel_velocities(0) * dt;
    joint_state_msg.position.data[1] += wheel_velocities(1) * dt;
    joint_state_msg.position.data[2] += wheel_velocities(2) * dt;
    joint_state_msg.position.data[3] += wheel_velocities(3) * dt;

    joint_state_msg.velocity.data[0] = wheel_velocities(0);
    joint_state_msg.velocity.data[1] = wheel_velocities(1);
    joint_state_msg.velocity.data[2] = wheel_velocities(2);
    joint_state_msg.velocity.data[3] = wheel_velocities(3);

    joint_state_msg.header.stamp.sec = synced_time_ms / 1000;
    joint_state_msg.header.stamp.nanosec = synced_time_ns;

    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));

    // Update the wanted joint state message

    Eigen::Vector4d wanted_wheel_velocities =
        robot_controller.get_set_wheel_velocities();

    wanted_joint_state_msg.velocity.data[0] = wanted_wheel_velocities(0);
    wanted_joint_state_msg.velocity.data[1] = wanted_wheel_velocities(1);
    wanted_joint_state_msg.velocity.data[2] = wanted_wheel_velocities(2);
    wanted_joint_state_msg.velocity.data[3] = wanted_wheel_velocities(3);

    wanted_joint_state_msg.header.stamp.sec = synced_time_ms / 1000;
    wanted_joint_state_msg.header.stamp.nanosec = synced_time_ns;

    RCSOFTCHECK(rcl_publish(&wanted_joint_state_publisher,
                            &wanted_joint_state_msg, NULL));

    delay(10);
}
