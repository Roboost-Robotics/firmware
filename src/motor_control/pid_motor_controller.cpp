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
#include "utils/comparisons.hpp"
#include <Arduino.h>

PIDMotorController::PIDMotorController(MotorDriver& motor_driver,
                                       Encoder& encoder, PIDController& pid,
                                       Filter& input_filter,
                                       Filter& output_filter, double min_output)
    : MotorController(motor_driver), encoder_(encoder), pid_(pid),
      input_filter_(input_filter), output_filter_(output_filter),
      min_output_(min_output)
{
}

void PIDMotorController::set_rotation_speed(float desired_rotation_speed)
{
    encoder_.update();

    double input = encoder_.get_velocity();

    // Serial.print(">PIDMotorController unfiltered input:");
    // Serial.println(input);

    input = input_filter_.update(input);

    double output = pid_.update(desired_rotation_speed, input);

    // Serial.print(">PIDMotorController unfiltered output:");
    // Serial.println(output);

    output = output_filter_.update(output);

    // If the output is too small and desired rotation speed is zero, set the
    // output to zero.
    if (std::abs(output) < min_output_ &&
        std::abs(desired_rotation_speed) < 1e-3)
    {
        output = 0.0;
    }

    motor_driver_.set_motor_control(output);

    // Serial.print(">PIDMotorController input:");
    // Serial.println(input);
    // Serial.print(">PIDMotorController output:");
    // Serial.println(output);
    // Serial.print(">PIDMotorController setpoint:");
    // Serial.println(desired_rotation_speed);
}

float PIDMotorController::get_rotation_speed()
{
    return encoder_.get_velocity();
}