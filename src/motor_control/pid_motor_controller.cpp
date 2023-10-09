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
                                       Encoder& encoder, double kp, double ki,
                                       double kd,
                                       const double control_upper_limit,
                                       const double control_lower_limit)
    : kp_(kp), ki_(ki), kd_(kd), control_upper_limit_(control_upper_limit),
      control_lower_limit_(control_lower_limit), MotorController(motor_driver),
      encoder_(encoder),
      pid_(&input_, &output_, &setpoint_, kp_, ki_, kd_, DIRECT)
{
    pid_.SetMode(AUTOMATIC);
    pid_.SetTunings(kp_, ki_, kd_);
    pid_.SetOutputLimits(control_lower_limit_, control_upper_limit_);

    // output_history_.assign(filter_window_size_, 0.0);
}

void PIDMotorController::set_rotation_speed(float desired_rotation_speed)
{
    encoder_.update();

    setpoint_ = desired_rotation_speed;
    input_ = encoder_.get_velocity();
    // input_ = filter_input(input_);

    // gain_scheduling();

    pid_.Compute();

    double control = output_;

    // double control = filter_output(output_);
    // apply_anti_windup(control);

    if (control > control_upper_limit_)
        control = control_upper_limit_;
    else if (control < control_lower_limit_)
        control = control_lower_limit_;

    if (print_debug_)
    {
        Serial.print(">setpoint:");
        Serial.println(setpoint_);
        Serial.print(">input:");
        Serial.println(input_);
        Serial.print(">output:");
        Serial.println(control);
        Serial.print(">kp:");
        Serial.println(pid_.GetKp());
        Serial.print(">ki:");
        Serial.println(pid_.GetKi());
        Serial.print(">kd:");
        Serial.println(pid_.GetKd());
    }
    Serial.println(control);

    motor_driver_.set_motor_control(control);
}

float PIDMotorController::get_rotation_speed()
{
    return encoder_.get_velocity();
}