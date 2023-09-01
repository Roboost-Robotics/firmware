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
                                       Encoder& encoder)
    : MotorController(motor_driver), encoder_(encoder)
{
}

void PIDMotorController::set_rotation_speed(float desired_rotation_speed)
{
    motor_driver_.set_motor_control(0);
}
