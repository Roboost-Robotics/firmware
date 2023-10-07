/**
 * @file core.cpp
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief This file contains the main functionality for controlling a mecanum
 * robot using micro-ROS via UDP.
 * @version 1.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 * @todo Refactor the code to use ROS data types instead of Eigen.
 * @todo Add joint state controller (similar to velocity controller).
 *
 */

#include <Arduino.h>
#include <ArduinoEigen.h>
#include <micro_ros_platformio.h>

#include "rcl_checks.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>

#include "conf_hardware.h"
#include "conf_network.h"
#include "motor-control/encoder.hpp"
#include "motor-control/motor-drivers/l298n_motor_driver.hpp"
#include "motor-control/pid_motor_controller.hpp"
#include "motor-control/simple_motor_controller.hpp"
#include "velocity_controller.hpp"

L298NMotorDriver driver_M0(M0_IN1, M0_IN2, M0_ENA, M0_PWM_CNL);
L298NMotorDriver driver_M1(M1_IN1, M1_IN2, M1_ENA, M1_PWM_CNL);
L298NMotorDriver driver_M2(M2_IN1, M2_IN2, M2_ENA, M2_PWM_CNL);
L298NMotorDriver driver_M3(M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL);

// Encoder direction needs to be reversed, as the encoder sits on the mirrored
// side of the motor
HalfQuadEncoder encoder_M0(M0_ENC_A, M0_ENC_B, M0_ENC_RESOLUTION);
HalfQuadEncoder encoder_M1(M1_ENC_A, M1_ENC_B, M1_ENC_RESOLUTION, true);
HalfQuadEncoder encoder_M2(M2_ENC_A, M2_ENC_B, M2_ENC_RESOLUTION);
HalfQuadEncoder encoder_M3(M3_ENC_A, M3_ENC_B, M3_ENC_RESOLUTION, true);

PIDMotorController controller_M0(driver_M0, encoder_M0);
PIDMotorController controller_M1(driver_M1, encoder_M1);
PIDMotorController controller_M2(driver_M2, encoder_M2);
PIDMotorController controller_M3(driver_M3, encoder_M3);

MotorControllerManager motor_control_manager{
    {&controller_M0, &controller_M1, &controller_M2, &controller_M3}};

MecanumKinematics4W kinematics(WHEEL_RADIUS, WHEEL_BASE, TRACK_WIDTH);
VelocityController robot_controller(motor_control_manager, &kinematics);

rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t odom_publisher;
// rcl_publisher_t joint_state_publisher;
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;
// sensor_msgs__msg__JointState joint_state_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Delta time for odometry calculation
unsigned long last_time = 0;
// Pose part of odometry
Eigen::Vector3d pose = Eigen::Vector3d::Zero();

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

    // Convert the ROS Twist message to an Eigen::Matrix<double, 3, 1>
    Eigen::Matrix<double, 3, 1> cmd;
    cmd(0) = msg->linear.x;
    cmd(1) = msg->linear.y;
    cmd(2) = msg->angular.z;

    robot_controller.set_latest_command(cmd);
}

/**
 * @brief Prints the current free heap and stack size to the serial monitor.
 *
 */
void inline print_debug_info()
{
    Serial.print("Free heap: ");
    Serial.print(xPortGetFreeHeapSize());
    Serial.print(", Free stack: ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));
}

/**
 * @brief Setup function for initializing micro-ROS, pin modes, etc.
 *
 */
void setup()
{
    // Configure serial transport
    Serial.begin(115200); // disable in production

    IPAddress agent_ip(AGENT_IP);
    uint16_t agent_port = AGENT_PORT;

    set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip,
                                 agent_port);
    delay(2000);

    allocator = rcl_get_default_allocator();

    // create init_options
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
        Serial.println("Failed to create init options, retrying...");
        print_debug_info();
        delay(1000);
    }

    while (rclc_node_init_default(&node, "roboost_core_node", "", &support) !=
           RCL_RET_OK)
    {
        Serial.println("Failed to create node, retrying...");
        print_debug_info();
        delay(1000);
    }

    while (rclc_publisher_init_default(
               &odom_publisher, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
               "odom") != RCL_RET_OK)
    {
        Serial.println("Failed to create odom publisher, retrying...");
        print_debug_info();
        delay(1000);
    }

    // while (rclc_publisher_init_default(
    //            &joint_state_publisher, &node,
    //            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    //            "joint_states") != RCL_RET_OK)
    // {
    //     Serial.println("Failed to create joint_state publisher,
    //     retrying..."); print_debug_info(); delay(1000);
    // }

    while (rclc_subscription_init_default(
               &cmd_vel_subscriber, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
               "cmd_vel") != RCL_RET_OK)
    {
        Serial.println("Failed to create cmd_vel subscriber, retrying...");
        print_debug_info();
        delay(1000);
    }

    // 2 is the number of subscriptions that will be added to the executor.
    while (rclc_executor_init(&executor, &support.context, 2, &allocator) !=
           RCL_RET_OK)
    {
        Serial.println("Failed to create executor, retrying...");
        print_debug_info();
        delay(1000);
    }

    while (rclc_executor_add_subscription(
               &executor, &cmd_vel_subscriber, &twist_msg,
               &cmd_vel_subscription_callback, ON_NEW_DATA) != RCL_RET_OK)
    {
        Serial.println(
            "Failed to add cmd_vel subscriber to executor,retrying...");
        print_debug_info();
        delay(1000);
    }

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Initialize the odometry message
    odom_msg.header.frame_id.data = "odom";
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
    odom_msg.child_frame_id.data = "base_link";
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
    odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;
}

/**
 * @brief Main loop for continuously updating and publishing the robot's
 * odometry.
 *
 */
void loop()
{
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    robot_controller.update();

    Eigen::Vector3d robot_velocity = robot_controller.get_robot_velocity();

    // Calculate the delta time for odometry calculation
    unsigned long now = millis();
    double dt = (now - last_time) / 1000.0;
    last_time = now;

    pose(0) += robot_velocity(0) * cos(pose(2)) * dt -
               robot_velocity(1) * sin(pose(2)) * dt;
    pose(1) += robot_velocity(0) * sin(pose(2)) * dt +
               robot_velocity(1) * cos(pose(2)) * dt;
    pose(2) += robot_velocity(2) * dt;
    pose(2) = atan2(sin(pose(2)), cos(pose(2)));

    odom_msg.pose.pose.position.x = pose(0);
    odom_msg.pose.pose.position.y = pose(1);
    // Orientation in quaternion notation
    odom_msg.pose.pose.orientation.w = cos(pose(2) / 2.0);
    odom_msg.pose.pose.orientation.z = sin(pose(2) / 2.0);

    odom_msg.twist.twist.linear.x = robot_velocity(0);
    odom_msg.twist.twist.linear.y = robot_velocity(1);
    odom_msg.twist.twist.angular.z = robot_velocity(2);

    // Print pose in Teleoplot format:
    Serial.print(">theta:");
    Serial.println(pose(2));

    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    delay(10);
}
