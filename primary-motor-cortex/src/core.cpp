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
#include <micro_ros_platformio.h>

#include "rcl_checks.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include "conf_hardware.h"
#include "conf_network.h"
#include "motor-control/motor-drivers/l298n_motor_driver.hpp"
#include "motor-control/simple_motor_controller.hpp"
#include "robot_controller.hpp"

L298NMotorDriver driver_M0(M0_IN1, M0_IN2, M0_ENA, M0_PWM_CNL);
L298NMotorDriver driver_M1(M1_IN1, M1_IN2, M1_ENA, M1_PWM_CNL);
L298NMotorDriver driver_M2(M2_IN1, M2_IN2, M2_ENA, M2_PWM_CNL);
L298NMotorDriver driver_M3(M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL);

SimpleMotorController controller_M0(driver_M0, 1.);
SimpleMotorController controller_M1(driver_M1, 1.);
SimpleMotorController controller_M2(driver_M2, 1.);
SimpleMotorController controller_M3(driver_M3, 1.);

MotorControllerManager motor_controll_manager{
    {&controller_M0, &controller_M1, &controller_M2,
     &controller_M3}}; // initializer list

// todo initialize kinematics
MecanumKinematics4W kinematics(WHEELRADIUS, WHEEL_BASE, TRACK_WIDTH);
RobotController robot_controller(motor_controll_manager, kinematics);

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

    // Convert the ROS Twist message to a BLA::Matrix<3>
    BLA::Matrix<3> cmd;
    cmd(0) = msg->linear.x;
    cmd(1) = msg->linear.y;
    cmd(2) = msg->angular.z;

    // Serial.print("Linear X: ");
    // Serial.println(cmd(0));
    // Serial.print("Linear Y: ");
    // Serial.println(cmd(1));
    // Serial.print("Angular Z: ");
    // Serial.println(cmd(2));

    // Print wheel speeds
    // Serial.print("Wheel speed M0: ");
    // Serial.println(motor_controll_manager.get_motor_speed(0));
    // Serial.print("Wheel speed M1: ");
    // Serial.println(motor_controll_manager.get_motor_speed(1));
    // Serial.print("Wheel speed M2: ");
    // Serial.println(motor_controll_manager.get_motor_speed(2));
    // Serial.print("Wheel speed M3: ");
    // Serial.println(motor_controll_manager.get_motor_speed(3));

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
    // Serial.println("Creating init options...");
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
        // Serial.println("Failed to create init options, retrying...");
        delay(1000); // Wait for 1 second
    }

    // Serial.println("Creating node...");
    while (rclc_node_init_default(&node, "roboost_core_node", "", &support) !=
           RCL_RET_OK)
    {
        // Serial.println("Failed to create node, retrying...");
        delay(1000); // Wait for 1 second
    }

    // Serial.print("Creating publisher...");
    while (rclc_publisher_init_default(
               &publisher, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
               "odom") != RCL_RET_OK)
    {
        // Serial.println("Failed to create publisher, retrying...");
        delay(1000); // Wait for 1 second
    }

    // Serial.println("Creating subscriber...");
    while (rclc_subscription_init_default(
               &subscriber, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
               "cmd_vel") != RCL_RET_OK)
    {
        // Serial.println("Failed to create subscriber, retrying...");
        delay(1000); // Wait for 1 second
    }

    // Serial.println("Creating executor...");
    while (rclc_executor_init(&executor, &support.context, 1, &allocator) !=
           RCL_RET_OK)
    {
        // Serial.println("Failed to create executor, retrying...");
        delay(1000); // Wait for 1 second
    }

    // Serial.println("Adding subscriber to executor...");
    while (rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                          &cmd_vel_subscription_callback,
                                          ON_NEW_DATA) != RCL_RET_OK)
    {
        // Serial.println("Failed to add subscriber to executor, retrying...");
        delay(1000); // Wait for 1 second
    }

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

/**
 * @brief Main loop for continuously updating and publishing robot's odometry.
 *
 */
void loop()
{
    // Publish the RobotController's latest odometry
    robot_controller.update();
    BLA::Matrix<6> odometry = robot_controller.get_odometry();

    // Convert the odometry matrix to a nav_msgs__msg__Odometry
    odom.pose.pose.position.x = odometry(0); // x position
    odom.pose.pose.position.y = odometry(1); // y position
    odom.pose.pose.orientation.z =
        odometry(2); // yaw orientation (using z-axis rotation)

    odom.twist.twist.linear.x = odometry(3);  // x linear velocity
    odom.twist.twist.linear.y = odometry(4);  // y linear velocity
    odom.twist.twist.angular.z = odometry(5); // z angular velocity

    RCSOFTCHECK(rcl_publish(&publisher, &odom, NULL));

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(10);
}