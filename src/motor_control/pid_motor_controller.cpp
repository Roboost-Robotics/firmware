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

    output_history_.assign(filter_window_size_, 0.0);
}

void PIDMotorController::set_rotation_speed(float desired_rotation_speed)
{
    encoder_.update();

    setpoint_ = desired_rotation_speed;
    input_ = encoder_.get_velocity();
    input_ = filter_input(input_);

    gain_scheduling();

    pid_.Compute();

    double control = filter_output(output_);
    apply_anti_windup(control);

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

// Method to check if two floats are approximately equal
// TODO: Move to utils
bool approx_equal(float a, float b, float epsilon)
{
    return std::abs(a - b) < epsilon;
}

double PIDMotorController::filter_output(double output)
{
    output_history_.push_front(output);

    if (output_history_.size() > filter_window_size_)
        output_history_.pop_back();

    if (approx_equal(setpoint_, 0.0, 0.01) && approx_equal(input_, 0.0, 0.01))
        return 0.0;

    double sum = 0.0;
    for (double value : output_history_)
        sum += value;

    return sum / output_history_.size();
}

double PIDMotorController::filter_input(double input)
{
    input_history_.push_front(input);

    if (input_history_.size() > filter_window_size_)
        input_history_.pop_back();

    double sum = 0.0;
    for (double value : input_history_)
        sum += value;

    return sum / input_history_.size();
}

void PIDMotorController::apply_anti_windup(float control)
{
    if (control > control_upper_limit_)
        pid_.SetTunings(kp_, 0.0, kd_);
    else if (control < control_lower_limit_)
        pid_.SetTunings(kp_, 0.0, kd_);
    else if (pid_.GetKi() == 0.0)
        pid_.SetTunings(kp_, ki_, kd_);
}

void PIDMotorController::gain_scheduling()
{
    double error = abs(setpoint_ - input_);
    if (error > 10.0)
        pid_.SetTunings(kp_ * 0.5, ki_ * 0.5, kd_);
    else if (error > 5.0)
        pid_.SetTunings(kp_ * 0.75, ki_ * 0.75, kd_);
    else
        pid_.SetTunings(kp_, ki_, kd_);
}