/**
 * @file pid.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Implementation of a Proportional-Integral-Derivative (PID) controller
 * for versitile usage.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "motor-control/pid.hpp"

PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0), previous_error_(0)
{
}

float PIDController::calculate(float setpoint, float actual_value)
{
    float error = setpoint - actual_value;
    integral_ += error;
    float derivative = error - previous_error_;
    previous_error_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}
