#ifndef ROS_HANDLER_H
#define ROS_HANDLER_H

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include "robot_controller.hpp"

extern RobotController* robot_controller_ptr;

// Global Variables (instead of private members)
extern rcl_subscription_t subscriber_;
extern rcl_publisher_t publisher_;
extern geometry_msgs__msg__Twist msg_;
extern nav_msgs__msg__Odometry odom_;

extern rclc_executor_t executor_;
extern rclc_support_t support_;
extern rcl_allocator_t allocator_;
extern rcl_node_t node_;

// Function Prototypes (instead of member functions)
void setup_ros_handler();
void spin_ros_handler();

// Callback function
void cmd_vel_subscription_callback(const void* msgin, void* ros_handler_void);

void set_robot_command(const BLA::Matrix<3>& cmd);

#endif // ROS_HANDLER_H
