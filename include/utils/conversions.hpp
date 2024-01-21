#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

#include <ArduinoEigen.h>
#include <geometry_msgs/msg/quaternion.h>

geometry_msgs__msg__Quaternion toQuaternion(double roll, double pitch,
                                            double yaw)
{
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    geometry_msgs__msg__Quaternion result;
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();
    result.w = q.w();
    return result;
}

#endif // CONVERSIONS_HPP