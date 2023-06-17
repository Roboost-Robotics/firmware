#ifndef ODOMETRY_PUBLISHER_H
#define ODOMETRY_PUBLISHER_H

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>

class OdometryPublisher {
public:
  OdometryPublisher(rcl_node_t& node, const char* topic_name);

  void publishOdometry(float linear_velocity, float angular_velocity);

private:
  rcl_publisher_t publisher_;
  nav_msgs__msg__Odometry msg_;
};

#endif // ODOMETRY_PUBLISHER_H