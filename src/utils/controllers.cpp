/**
 * @file controllers.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility function and class definitions for controllers.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "utils/controllers.hpp"

PIDController::PIDController(double kp, double ki, double kd,
                             double max_expected_sampling_time)
    : kp_(kp), ki_(ki), kd_(kd),
      max_expected_sampling_time_(max_expected_sampling_time), integral_(0.0),
      previous_error_(0.0),
      derivative_filter_(1.0 /
                             (1.0 + 2.0 * PI * kd * max_expected_sampling_time),
                         max_expected_sampling_time)
{
    last_update_time_ = micros();
}

double PIDController::update(double setpoint, double input)
{
    double sampling_time = (micros() - last_update_time_) * 1e-6;
    last_update_time_ = micros();
    double error = setpoint - input;

    integral_ += error * sampling_time;
    double derivative =
        derivative_filter_.update((error - previous_error_) / sampling_time);
    previous_error_ = error;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    return output;
}

void PIDController::reset()
{
    integral_ = 0.0;
    previous_error_ = 0.0;
    derivative_filter_.reset();
}

double PIDController::get_kp() { return kp_; }

double PIDController::get_ki() { return ki_; }

double PIDController::get_kd() { return kd_; }

void PIDController::set_kp(double kp) { kp_ = kp; }

void PIDController::set_ki(double ki) { ki_ = ki; }

void PIDController::set_kd(double kd) { kd_ = kd; }