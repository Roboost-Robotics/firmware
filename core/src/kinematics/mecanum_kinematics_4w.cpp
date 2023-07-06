/**
 * @file mecanum_kinematics_4w.cpp //TODO
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "kinematics/kinematics.hpp"

MecanumKinematics4W::MecanumKinematics4W(double wheel_radius, double wheel_base, double track_width)
: wheel_radius_(wheel_radius), wheel_base_(wheel_base), track_width_(track_width) {}

BLA::Matrix<3> MecanumKinematics4W::calculate_robot_velocity(const BLA::Matrix<4>& wheel_velocity) {
    BLA::Matrix<3> robot_velocity;

    BLA::Matrix<3, 4> inverseKinematicsModel = { 1, 1, 1, 1, 
                                                -1, 1, 1, -1, 
                                                -1/(wheel_base_ + track_width_), 1/(wheel_base_ + track_width_), -1/(wheel_base_ + track_width_), 1/(wheel_base_ + track_width_)};

    robot_velocity = inverseKinematicsModel * wheel_velocity;
    robot_velocity *= wheel_radius_ / 4;

    return robot_velocity;
}

BLA::Matrix<4> MecanumKinematics4W::calculate_wheel_velocity(const BLA::Matrix<3>& robot_velocity) {
    BLA::Matrix<4> wheel_velocity;
    BLA::Matrix<4, 3> forwardKinematicsModel = { 1, -1, -(wheel_base_ + track_width_),
                                                1, 1, wheel_base_ + track_width_,
                                                1, 1, -(wheel_base_ + track_width_),
                                                1, -1, wheel_base_ + track_width_};
    wheel_velocity = forwardKinematicsModel * robot_velocity;
    wheel_velocity *=  1 / wheel_radius_;

    return wheel_velocity;
}
