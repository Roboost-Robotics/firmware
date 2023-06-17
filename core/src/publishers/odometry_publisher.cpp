#include "publishers/odometry_publisher.hpp"

OdometryPublisher::OdometryPublisher(rcl_node_t& node, const char* topic_name) {
  rclc_publisher_init_default(
    &publisher_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    topic_name
  );

  // Initialize the odometry message
  msg_ = nav_msgs__msg__Odometry__create();
}

void OdometryPublisher::publishOdometry(float linear_velocity, float angular_velocity) {
  // Assume you have a way to calculate the current pose of the robot
  // Fill in the pose and velocity data in the odometry message
  msg_->twist.twist.linear.x = linear_velocity;
  msg_->twist.twist.angular.z = angular_velocity;

  rcl_publish(&publisher_, &msg_, NULL);
}
