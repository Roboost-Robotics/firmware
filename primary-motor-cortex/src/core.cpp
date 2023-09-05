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

#include "conf_hardware.h"
#include "conf_network.h"
#include "motor-control/encoder.hpp"
#include "motor-control/motor-drivers/l298n_motor_driver.hpp"
#include "motor-control/pid_motor_controller.hpp"
#include "motor-control/simple_motor_controller.hpp"
#include "robot_controller.hpp"

L298NMotorDriver driver_M0(M0_IN1, M0_IN2, M0_ENA, M0_PWM_CNL);
L298NMotorDriver driver_M1(M1_IN1, M1_IN2, M1_ENA, M1_PWM_CNL);
L298NMotorDriver driver_M2(M2_IN1, M2_IN2, M2_ENA, M2_PWM_CNL);
L298NMotorDriver driver_M3(M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL);

HalfQuadEncoder encoder_M0(M0_ENC_A, M0_ENC_B, M0_ENC_RESOLUTION);
HalfQuadEncoder encoder_M1(M1_ENC_A, M1_ENC_B, M1_ENC_RESOLUTION);
HalfQuadEncoder encoder_M2(M2_ENC_A, M2_ENC_B, M2_ENC_RESOLUTION);
HalfQuadEncoder encoder_M3(M3_ENC_A, M3_ENC_B, M3_ENC_RESOLUTION);

PIDMotorController controller_M0(driver_M0, encoder_M0);
PIDMotorController controller_M1(driver_M1, encoder_M1);
PIDMotorController controller_M2(driver_M2, encoder_M2);
PIDMotorController controller_M3(driver_M3, encoder_M3);

// SimpleMotorController controller_M0(driver_M0, 0.5);
// SimpleMotorController controller_M1(driver_M1, 0.5);
// SimpleMotorController controller_M2(driver_M2, 0.5);
// SimpleMotorController controller_M3(driver_M3, 0.5);

MotorControllerManager motor_control_manager{
    {&controller_M0, &controller_M1, &controller_M2, &controller_M3}}; // initializer list

MecanumKinematics4W kinematics(WHEEL_RADIUS, WHEEL_BASE, TRACK_WIDTH);
RobotController robot_controller(motor_control_manager, &kinematics);

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg;
nav_msgs__msg__Odometry odom;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

/**
 * @brief Callback function for handling incoming cmd_vel (velocity command)
 * messages.
 *
 * @param msgin Pointer to the received geometry_msgs__msg__Twist message.
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
 * @brief Setup function for initializing micro-ROS, pin modes, etc.
 *
 */
void setup()
{
    // Configure serial transport
    Serial.begin(115200); // disable in production

    IPAddress agent_ip(AGENT_IP);
    uint16_t agent_port = AGENT_PORT;

    set_microros_wifi_transports(SSID, SSID_PW, agent_ip, agent_port);
    delay(2000);

    allocator = rcl_get_default_allocator();

    // create init_options
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
        // Serial.println("Failed to create init options, retrying...");
        delay(1000);
    }

    while (rclc_node_init_default(&node, "roboost_core_node", "", &support) != RCL_RET_OK)
    {
        // Serial.println("Failed to create node, retrying...");
        delay(1000);
    }

    while (rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                       "odom") != RCL_RET_OK)
    {
        // Serial.println("Failed to create publisher, retrying...");
        delay(1000);
    }

    while (rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                          "cmd_vel") != RCL_RET_OK)
    {
        // Serial.println("Failed to create subscriber, retrying...");
        delay(1000);
    }

    while (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
    {
        // Serial.println("Failed to create executor, retrying...");
        delay(1000);
    }

    while (rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_subscription_callback, ON_NEW_DATA) !=
           RCL_RET_OK)
    {
        // Serial.println("Failed to add subscriber to executor,retrying...");
        delay(1000);
    }

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

/**
 * @brief Main loop for continuously updating and publishing the robot's
 * odometry.
 *
 */
void loop()
{
    robot_controller.update();
    Eigen::Matrix<double, 6, 1> odometry = robot_controller.get_odometry();

    odom.pose.pose.position.x = odometry(0);
    odom.pose.pose.position.y = odometry(1);
    odom.pose.pose.orientation.z = odometry(2);

    odom.twist.twist.linear.x = odometry(3);
    odom.twist.twist.linear.y = odometry(4);
    odom.twist.twist.angular.z = odometry(5);

    RCSOFTCHECK(rcl_publish(&publisher, &odom, NULL));

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    delay(10);
}
