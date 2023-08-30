// /**
//  * @file core.cpp
//  * @author Friedl Jakob (friedl.jak@gmail.com)
//  * @brief This file contains the main functionality for controlling a mecanum
//  * robot using micro-ROS via UDP.
//  * @version 1.1
//  * @date 2023-08-21
//  *
//  * @copyright Copyright (c) 2023
//  *
//  */

// #include <Arduino.h>
// #include <ArduinoEigen.h>
// #include <micro_ros_platformio.h>

// #include "rcl_checks.h"
// #include <rcl/rcl.h>
// #include <rclc/executor.h>
// #include <rclc/rclc.h>

// #include <geometry_msgs/msg/twist.h>
// #include <nav_msgs/msg/odometry.h>

// #include "conf_hardware.h"
// #include "conf_network.h"
// #include "motor-control/encoder.hpp"
// #include "motor-control/motor-drivers/l298n_motor_driver.hpp"
// #include "motor-control/simple_motor_controller.hpp"
// #include "robot_controller.hpp"

// class TesetClass
// {
// public:
//     TesetClass(const int pin_A, const int pin_B, const int resolution)
//         : pin_A_(pin_A), pin_B_(pin_B), resolution_(resolution)
//     {
//         instancePtr = this;
//     }

//     void setup()
//     {
//         pinMode(pin_A_, INPUT);
//         pinMode(pin_B_, INPUT);
//         attachInterrupt(digitalPinToInterrupt(pin_A_), instanceISRWrapper_pinA,
//                         RISING, this);
//         attachInterrupt(digitalPinToInterrupt(pin_B_), instanceISRWrapper_pinB,
//                         RISING, this);

//         last_time_ = micros();
//     }

//     void printCount()
//     {
//         noInterrupts();
//         int count_A_local = count_A;
//         int count_B_local = count_B;
//         count_A = 0;
//         count_B = 0;
//         interrupts();

//         int direction = (count_B_local >= 0) ? 1 : -1;
//         int dt = micros() - last_time_;
//         last_time_ = micros();

//         float velocity = (direction * resolution_ * count_A_local) / (float)dt;
//         Serial.println(velocity);
//     }

// private:
//     int count_A = 0;
//     int count_B = 0;
//     int pin_A_;
//     int pin_B_;
//     int resolution_;
//     unsigned long last_time_;
//     TesetClass* instancePtr; // Pointer to the instance

//     static void instanceISRWrapper_pinA(void* context)
//     {
//         TesetClass* instance = static_cast<TesetClass*>(context);
//         if (instance)
//             instance->instanceISR_pinA();
//     }

//     static void instanceISRWrapper_pinB(void* context)
//     {
//         TesetClass* instance = static_cast<TesetClass*>(context);
//         if (instance)
//             instance->instanceISR_pinB();
//     }

//     void IRAM_ATTR instanceISR_pinA() { count_A++; }
//     void IRAM_ATTR instanceISR_pinB() { count_B++; }
// };

// // TesetClass* TesetClass::instancePtr = nullptr;

// // TesetClass testObject1(5, 15, 360);
// // TesetClass testObject2(17, 16, 600);
// // TesetClass testObject3(35, 34, 360);
// TesetClass testObject4(39, 36, 360);

// L298NMotorDriver driver_M0(M0_IN1, M0_IN2, M0_ENA, M0_PWM_CNL);
// L298NMotorDriver driver_M1(M1_IN1, M1_IN2, M1_ENA, M1_PWM_CNL);
// L298NMotorDriver driver_M2(M2_IN1, M2_IN2, M2_ENA, M2_PWM_CNL);
// L298NMotorDriver driver_M3(M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL);

// // Encoder encoder_M0(EC_M0_A, EC_M0_B, EC_M0_RES);
// // Encoder encoder_M1(EC_M1_A, EC_M1_B, EC_M1_RES);
// // Encoder encoder_M2(EC_M2_A, EC_M2_B, EC_M2_RES);
// // Encoder encoder_M3(EC_M3_A, EC_M3_B, EC_M3_RES);

// SimpleMotorController controller_M0(driver_M0, 1.);
// SimpleMotorController controller_M1(driver_M1, 1.);
// SimpleMotorController controller_M2(driver_M2, 1.);
// SimpleMotorController controller_M3(driver_M3, 1.);

// MotorControllerManager motor_control_manager{
//     {&controller_M0, &controller_M1, &controller_M2,
//      &controller_M3}}; // initializer list

// MecanumKinematics4W kinematics(WHEEL_RADIUS, WHEEL_BASE, TRACK_WIDTH);
// RobotController robot_controller(motor_control_manager, &kinematics);

// rcl_subscription_t subscriber;
// rcl_publisher_t publisher;
// geometry_msgs__msg__Twist msg;
// nav_msgs__msg__Odometry odom;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;

// /**
//  * @brief Callback function for handling incoming cmd_vel (velocity command)
//  * messages.
//  *
//  * @param msgin Pointer to the received geometry_msgs__msg__Twist message.
//  */
// void cmd_vel_subscription_callback(const void* msgin)
// {
//     const auto* msg = reinterpret_cast<const geometry_msgs__msg__Twist*>(msgin);

//     // Convert the ROS Twist message to an Eigen::Matrix<double, 3, 1>
//     Eigen::Matrix<double, 3, 1> cmd;
//     cmd(0) = msg->linear.x;
//     cmd(1) = msg->linear.y;
//     cmd(2) = msg->angular.z;

//     robot_controller.set_latest_command(cmd);
// }

// /**
//  * @brief Setup function for initializing micro-ROS, pin modes, etc.
//  *
//  */
// void setup()
// {
//     // Configure serial transport
//     Serial.begin(115200); // disable in production

//     IPAddress agent_ip(AGENT_IP);
//     uint16_t agent_port = AGENT_PORT;

//     set_microros_wifi_transports(SSID, SSID_PW, agent_ip, agent_port);
//     delay(2000);

//     allocator = rcl_get_default_allocator();

//     // create init_options
//     while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
//     {
//         // Serial.println("Failed to create init options, retrying...");
//         delay(1000);
//     }

//     while (rclc_node_init_default(&node, "roboost_core_node", "", &support) !=
//            RCL_RET_OK)
//     {
//         // Serial.println("Failed to create node, retrying...");
//         delay(1000);
//     }

//     while (rclc_publisher_init_default(
//                &publisher, &node,
//                ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
//                "odom") != RCL_RET_OK)
//     {
//         // Serial.println("Failed to create publisher, retrying...");
//         delay(1000);
//     }

//     while (rclc_subscription_init_default(
//                &subscriber, &node,
//                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
//                "cmd_vel") != RCL_RET_OK)
//     {
//         // Serial.println("Failed to create subscriber, retrying...");
//         delay(1000);
//     }

//     while (rclc_executor_init(&executor, &support.context, 1, &allocator) !=
//            RCL_RET_OK)
//     {
//         // Serial.println("Failed to create executor, retrying...");
//         delay(1000);
//     }

//     while (rclc_executor_add_subscription(&executor, &subscriber, &msg,
//                                           &cmd_vel_subscription_callback,
//                                           ON_NEW_DATA) != RCL_RET_OK)
//     {
//         // Serial.println("Failed to add subscriber to executor,retrying...");
//         delay(1000);
//     }

//     // //! REMOVE AFTER TESTING
//     // pinMode(pin_A_, INPUT_PULLUP); // Using internal pull-up resistors
//     // pinMode(pin_B_, INPUT_PULLUP);
//     // last_state_A_ = digitalRead(pin_A_);
//     // last_state_B_ = digitalRead(pin_B_);
//     // last_time_ = micros();

//     // attachInterrupt(digitalPinToInterrupt(pin_A_), function_ISR_EC_A,
//     // CHANGE); attachInterrupt(digitalPinToInterrupt(pin_B_),
//     // function_ISR_EC_B, CHANGE);
//     // //! REMOVE AFTER TESTING

//     delay(500);
//     pinMode(LED_BUILTIN, OUTPUT);
//     digitalWrite(LED_BUILTIN, HIGH);
// }

// /**
//  * @brief Main loop for continuously updating and publishing the robot's
//  * odometry.
//  *
//  */
// void loop()
// {
//     robot_controller.update();
//     Eigen::Matrix<double, 6, 1> odometry = robot_controller.get_odometry();

//     odom.pose.pose.position.x = odometry(0);
//     odom.pose.pose.position.y = odometry(1);
//     odom.pose.pose.orientation.z = odometry(2);

//     odom.twist.twist.linear.x = odometry(3);
//     odom.twist.twist.linear.y = odometry(4);
//     odom.twist.twist.angular.z = odometry(5);

//     RCSOFTCHECK(rcl_publish(&publisher, &odom, NULL));

//     RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

//     testObject4.printCount();

//     delay(10);
// }
