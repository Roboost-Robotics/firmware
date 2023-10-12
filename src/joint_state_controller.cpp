/**
 * @file joint_state_controller.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Implementation of the JointStateController class.
 * @version 1.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "joint_state_controller.hpp"

JointStateController::JointStateController(
    MotorControllerManager& motor_manager)
    : motor_manager_(motor_manager)
{
    latest_command_ = Eigen::VectorXd::Zero();
}

void JointStateController::update()
{
    int motor_count = motor_manager_.get_motor_count();
    if (latest_command_.size() != motor_count)
    {
        Serial.println("Not enough motor controllers");
        return;
    }

    for (int i = 0; i < motor_count; ++i)
    {
        motor_manager_.set_motor_speed(i, latest_command_(i));
    }
    motor_manager_.update();

    Eigen::VectorXd actual_wheel_speeds(motor_count);
    for (int i = 0; i < motor_count; ++i)
    {
        actual_wheel_speeds(i) = motor_manager_.get_motor_speed(i);
    }

    // TODO: Fill in the joint state message
}

void JointStateController::set_latest_command(
    const Eigen::VectorXd& latest_command)
{
    latest_command_ = latest_command;
}

sensor_msgs__msg__JointState JointStateController::get_joint_state_msg()
{
    return joint_state_msg_;
}
