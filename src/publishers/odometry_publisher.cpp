#include "publishers/odometry_publisher.hpp"

#include "rcl_checks.h"
#include "utils/conversions.hpp"
#include <cmath>
#include <cstring>
#include <rcl/rcl.h>
#include <rclc/publisher.h>

extern rcl_time_point_value_t global_current_time;

OdometryPublisher::OdometryPublisher()
{
    std::fill(std::begin(covariance_), std::end(covariance_), 0.0);
    frame_id_[0] = '\\0';
    child_frame_id_[0] = '\\0';
    pose_ = Eigen::Vector3d::Zero();
}

void OdometryPublisher::initialize(rcl_node_t* node_handle,
                                   const char* odom_frame_id,
                                   const char* child_frame_id,
                                   const char* odom_topic)
{
    strcpy(frame_id_, odom_frame_id);
    strcpy(child_frame_id_, child_frame_id);

    odom_msg_.header.frame_id.data = frame_id_;
    odom_msg_.header.frame_id.size = strlen(frame_id_);
    odom_msg_.header.frame_id.capacity = sizeof(frame_id_);
    odom_msg_.child_frame_id.data = child_frame_id_;
    odom_msg_.child_frame_id.size = strlen(child_frame_id_);
    odom_msg_.child_frame_id.capacity = sizeof(child_frame_id_);

    for (size_t i = 0; i < 36; ++i)
    {
        odom_msg_.pose.covariance[i] = covariance_[i];
        odom_msg_.twist.covariance[i] = covariance_[i];
    }

    RCCHECK(rclc_publisher_init_default(
        &odom_publisher_, node_handle,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), odom_topic));
}

void OdometryPublisher::update(const Eigen::Vector3d& robot_velocity)
{
    double dt =
        (global_current_time - odom_msg_.header.stamp.sec * 1000000000LL -
         odom_msg_.header.stamp.nanosec) /
        1000000000.0; // Convert to seconds

    pose_(0) += robot_velocity(0) * cos(pose_(2)) * dt -
                robot_velocity(1) * sin(pose_(2)) * dt;
    pose_(1) += robot_velocity(0) * sin(pose_(2)) * dt +
                robot_velocity(1) * cos(pose_(2)) * dt;
    pose_(2) += robot_velocity(2) * dt;
    pose_(2) = atan2(sin(pose_(2)), cos(pose_(2))); // Normalize the angle

    odom_msg_.pose.pose.position.x = pose_(0);
    odom_msg_.pose.pose.position.y = pose_(1);
    odom_msg_.pose.pose.orientation = toQuaternion(0, 0, pose_(2));

    odom_msg_.twist.twist.linear.x = robot_velocity(0);
    odom_msg_.twist.twist.linear.y = robot_velocity(1);
    odom_msg_.twist.twist.angular.z = robot_velocity(2);

    odom_msg_.header.stamp.sec = global_current_time / 1000000000LL;
    odom_msg_.header.stamp.nanosec = global_current_time % 1000000000LL;
}

void OdometryPublisher::publish()
{
    if (rcl_publish(&odom_publisher_, &odom_msg_, nullptr) != RCL_RET_OK)
    {
        // TODO Handle the error accordingly
    }
}