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

MecanumKinematics4W::MecanumKinematics4W(const float& wheel_radius,
                                         const float& wheel_base,
                                         const float& track_width)
    : wheel_radius_(wheel_radius), wheel_base_(wheel_base),
      track_width_(track_width)
{

    // r is the distance from the turning center to the wheel
    // in the current configuration, r is always in the center of the robot
    // TODO: dynamic r
    const double r =
        wheel_base / 2.0 + track_width / 2.0; // sqrt(pow(wheel_base_ / 2.0, 2)
                                              // + pow(track_width_ / 2.0, 2));

    // clang-format off
    forward_kinematics_ << 1, -1, -r,
                           -1, -1, -r,
                           1, 1, -r,
                           -1, 1, -r;

    inverse_kinematics_ << 1, -1, -1, -1, 
                           -1, -1, 1, 1, 
                           -1/r, -1/r, -1/r, -1/r;
    // clang-format on
}

Eigen::VectorXd MecanumKinematics4W::calculate_wheel_velocity(
    const Eigen::Vector3d& robot_velocity)
{
    Eigen::VectorXd wheel_velocity(4);

    wheel_velocity = forward_kinematics_ * robot_velocity;
    wheel_velocity *= 1 / wheel_radius_;

    return wheel_velocity;
}

Eigen::Vector3d MecanumKinematics4W::calculate_robot_velocity(
    const Eigen::VectorXd& wheel_velocity)
{
    Eigen::Vector3d robot_velocity;

    robot_velocity = inverse_kinematics_ * wheel_velocity;
    robot_velocity *= wheel_radius_ / 4.0;

    return robot_velocity;
}