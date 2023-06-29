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

    // TODO: fix subscriber
// Forward declare the RosHandler class so that it can be used in the callback function prototype
class RosHandler;

    // TODO: fix subscriber
// Update the callback function prototype
void cmd_vel_subscription_callback(const void* msgin, void* ros_handler_void);

class RosHandler {
public:
    RosHandler(RobotController& robot_controller);
    ~RosHandler();

    void setup();
    void spin();

    // TODO: fix subscriber
    void set_robot_command(const BLA::Matrix<3>& cmd);

private:
    rcl_subscription_t subscriber_;
    rcl_publisher_t publisher_;
    geometry_msgs__msg__Twist msg_;
    nav_msgs__msg__Odometry odom_;

    rclc_executor_t executor_;
    rclc_support_t support_;
    rcl_allocator_t allocator_;
    rcl_node_t node_;

    RobotController& robot_controller_;
};

#endif // ROS_HANDLER_H
