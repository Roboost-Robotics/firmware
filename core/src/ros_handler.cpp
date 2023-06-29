// ros_handler.cpp
#include "ros_handler.hpp"

#include "conf_network.h"
#include "rcl_checks.h"

// TODO: maybe unnessesary includes
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>


RosHandler::RosHandler(RobotController& robot_controller)  // Update this constructor
    : robot_controller_(robot_controller) {
    // Constructor
}

RosHandler::~RosHandler() {
    // Destructor
}

void RosHandler::set_robot_command(const BLA::Matrix<3>& cmd){
    robot_controller_.set_latest_command(cmd);
}

    // TODO: fix subscriber
// Update the callback function definition
void cmd_vel_subscription_callback(const void* msgin, void* ros_handler_void) {
    RosHandler* ros_handler = static_cast<RosHandler*>(ros_handler_void);
    const auto* msg = reinterpret_cast<const geometry_msgs__msg__Twist*>(msgin);

    // Convert the ROS Twist message to a BLA::Matrix<3> and call set_latest_command
    BLA::Matrix<3> cmd;
    cmd(0) = msg->linear.x;
    cmd(1) = msg->linear.y;
    cmd(2) = msg->angular.z;

    ros_handler->set_robot_command(cmd);
}

void RosHandler::setup() {
    Serial.begin(115200);
    IPAddress agent_ip(AGENT_IP);
    size_t agent_port = AGENT_PORT;

    set_microros_wifi_transports(SSID, SSID_PW, agent_ip, agent_port);
    delay(2000);

    allocator_ = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));

    // create node
    RCCHECK(rclc_node_init_default(&node_, "roboost_core_node", "", &support_));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg_, Twist),
        "cmd_vel"
    ));

    // create executor
    RCCHECK(rclc_executor_init(&executor_, &support_.context, 1, &allocator_));

    // TODO: fix subscriber
    RCCHECK(rclc_executor_add_subscription(&executor_, &subscriber_, &msg_, cmd_vel_subscription_callback, this, ON_NEW_DATA));

    // Create publisher for odometry
    RCCHECK(rclc_publisher_init_default(
        &publisher_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"
    ));
}

void RosHandler::spin() {

    // Publish the RobotController's latest odometry
    BLA::Matrix<6> odometry = robot_controller_.get_odometry();

    // Convert the odometry matrix to a nav_msgs__msg__Odometry
    odom_.pose.pose.position.x = odometry(0); // x position
    odom_.pose.pose.position.y = odometry(1); // y position
    odom_.pose.pose.orientation.z = odometry(2); // yaw orientation (using z-axis rotation)
    
    odom_.twist.twist.linear.x = odometry(3); // x linear velocity
    odom_.twist.twist.linear.y = odometry(4); // y linear velocity
    odom_.twist.twist.angular.z = odometry(5); // z angular velocity

    RCSOFTCHECK(rcl_publish(&publisher_, &odom_, NULL));
    RCSOFTCHECK(rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100)));
}