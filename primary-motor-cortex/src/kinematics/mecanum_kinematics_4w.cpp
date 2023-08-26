/**
 * @file mecanum_kinematics_4w.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Implementation of MecanumKinematics4W class.
 * @version 1.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "kinematics/kinematics.hpp"

MecanumKinematics4W::MecanumKinematics4W(double wheel_radius, double wheel_base,
                                         double track_width)
    : wheel_radius_(wheel_radius), wheel_base_(wheel_base),
      track_width_(track_width)
{
}

Eigen::Vector3d MecanumKinematics4W::calculate_robot_velocity(
    const Eigen::VectorXd& wheel_velocity)
{
    Eigen::Vector3d robot_velocity;

    Eigen::Matrix<double, 3, 4> inverseKinematicsModel;
    // clang-format off
    inverseKinematicsModel << 1, 1, 1, 1, 
                             -1, 1, 1, -1, 
                             -1/(wheel_base_ + track_width_), 1/(wheel_base_ + track_width_), -1/(wheel_base_ + track_width_), 1/(wheel_base_ + track_width_);
    // clang-format on
    robot_velocity = inverseKinematicsModel * wheel_velocity;
    robot_velocity *= wheel_radius_ / 4.0;

    return robot_velocity;
}

Eigen::VectorXd MecanumKinematics4W::calculate_wheel_velocity(
    const Eigen::Vector3d& robot_velocity)
{
    Eigen::VectorXd wheel_velocity(4);

    Eigen::Matrix<double, 4, 3> forwardKinematicsModel;
    // clang-format off
    forwardKinematicsModel << 1, -1, -(wheel_base_ + track_width_),
                             1, 1, wheel_base_ + track_width_,
                             1, 1, -(wheel_base_ + track_width_),
                             1, -1, wheel_base_ + track_width_;
    // clang-format on

    wheel_velocity = forwardKinematicsModel * robot_velocity;
    wheel_velocity *= 1 / wheel_radius_;

    return wheel_velocity;
}
