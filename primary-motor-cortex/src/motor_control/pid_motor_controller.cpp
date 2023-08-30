/**
 * @file pid_motor_controller.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief This file contains the implementation of a PID motor controller.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "motor-control/pid_motor_controller.hpp"
#include <Arduino.h>
#include <algorithm>

PIDMotorController::PIDMotorController(MotorDriver& motor_driver,
                                       float max_rotation_speed,
                                       Encoder& encoder)
    : MotorController(motor_driver), max_rotation_speed_(max_rotation_speed),
      encoder_(encoder)
{
    attachInterrupt(
        digitalPinToInterrupt(encoder_.get_pin_A()),
        []() { encoder_.function_ISR_EC_A(); }, RISING);
    attachInterrupt(
        digitalPinToInterrupt(encoder_.get_pin_B()),
        []() { encoder_.function_ISR_EC_B(); }, RISING);
}

void PIDMotorController::set_rotation_speed(float desired_rotation_speed)
{
    desired_rotation_speed = std::clamp(
        desired_rotation_speed, -max_rotation_speed_, max_rotation_speed_);

    float control_value = desired_rotation_speed / max_rotation_speed_;

    motor_driver_.set_motor_control(control_value);
}
