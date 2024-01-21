/**
 * @file velocity_controller.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Implementation of the VelocityController class.
 * @version 1.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "velocity_controller.hpp"

VelocityController::VelocityController(MotorControllerManager& motor_manager,
                                       Kinematics* kinematics_model)
    : motor_manager_(motor_manager), kinematics_model_(kinematics_model)
{
    // Initialize latest_command and odometry_ to default values here
    latest_command_ = Eigen::Vector3d::Zero();
    robot_velocity_ = Eigen::Vector3d::Zero();
}

void VelocityController::update()
{
    Eigen::VectorXd desired_wheel_speeds =
        kinematics_model_->calculate_wheel_velocity(latest_command_);

    int motor_count = motor_manager_.get_motor_count();
    if (desired_wheel_speeds.size() != motor_count)
    {
        Serial.println("Not enough motor controllers");
        return;
    }

    for (int i = 0; i < motor_count; ++i)
    {
        motor_manager_.set_motor_speed(i, desired_wheel_speeds(i));
    }
    motor_manager_.update();

    Eigen::VectorXd actual_wheel_speeds(motor_count);
    for (int i = 0; i < motor_count; ++i)
    {
        actual_wheel_speeds(i) = motor_manager_.get_motor_speed(i);
    }

    robot_velocity_ =
        kinematics_model_->calculate_robot_velocity(actual_wheel_speeds);
}

Eigen::Vector3d VelocityController::get_robot_velocity()
{
    // Return the latest odometry data
    return robot_velocity_;
}

Eigen::VectorXd VelocityController::get_set_wheel_velocities()
{
    // Return the latest set wheel velocities
    return kinematics_model_->calculate_wheel_velocity(latest_command_);
}

void VelocityController::set_latest_command(
    const Eigen::Vector3d& latest_command)
{
    latest_command_ = latest_command;
}
