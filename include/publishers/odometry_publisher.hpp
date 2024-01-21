#ifndef ODOMETRY_PUBLISHER_H
#define ODOMETRY_PUBLISHER_H

#include <ArduinoEigen.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>

extern rcl_time_point_value_t global_current_time;

class OdometryPublisher
{
public:
    OdometryPublisher();
    void initialize(rcl_node_t* node_handle, const char* odom_frame_id,
                    const char* child_frame_id, const char* odom_topic);
    void update(const Eigen::Vector3d& robot_velocity);
    void publish();

private:
    rcl_publisher_t odom_publisher_;
    nav_msgs__msg__Odometry odom_msg_;
    Eigen::Vector3d pose_;
    double covariance_[36];
    char frame_id_[20];
    char child_frame_id_[20];
};

#endif // ODOMETRY_PUBLISHER_H
