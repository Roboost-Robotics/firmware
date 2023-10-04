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

PIDMotorController::PIDMotorController(MotorDriver& motor_driver, Encoder& encoder, const double kp, const double ki,
                                       const double kd)
    : kp_(kp), ki_(ki), kd_(kd), MotorController(motor_driver), encoder_(encoder),
      pid_(&input_, &output_, &setpoint_, kp_, ki_, kd_, DIRECT)
{
    pid_.SetMode(AUTOMATIC);
    pid_.SetTunings(kp_, ki_, kd_);
    pid_.SetOutputLimits(-1, 1);
}

void PIDMotorController::set_rotation_speed(float desired_rotation_speed)
{
    encoder_.update();

    setpoint_ = desired_rotation_speed;
    input_ = encoder_.get_velocity();

    pid_.Compute();

    float control = output_;

    // if abs(control) < 0.1, set control to 0
    if (abs(control) < 0.1)
    {
        control = 0;
    }

    motor_driver_.set_motor_control(control);

    // // Print output, input and setpoint values
    // Serial.print("output_: ");
    // Serial.print(output_);
    // Serial.print(" input_: ");
    // Serial.print(input_);
    // Serial.print(" desired_rotation_speed: ");
    // Serial.println(desired_rotation_speed);
}

float PIDMotorController::get_rotation_speed() { return encoder_.get_velocity(); }