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

using namespace roboost::controllers;
using namespace roboost::timing;

PIDController::PIDController(double kp, double ki, double kd, double max_expected_sampling_time, double max_integral, TimingService& timing_service)
    : kp_(kp), ki_(ki), kd_(kd), max_expected_sampling_time_(max_expected_sampling_time), integral_(0.0), previous_error_(0.0),
      derivative_filter_(1.0 / (1.0 + 2.0 * PI * kd * max_expected_sampling_time), max_expected_sampling_time), max_integral_(max_integral), timing_service_(timing_service)
{
}

double PIDController::update(double setpoint, double input)
{
    double dt = MICROS_TO_SECONDS_DOUBLE(timing_service_.getDeltaTime());
    double error = setpoint - input;

    integral_ += error * dt;

    // Enforce the maximum integral limit
    if (integral_ > max_integral_)
    {
        integral_ = max_integral_;
    }
    else if (integral_ < -max_integral_)
    {
        integral_ = -max_integral_;
    }
    double derivative = derivative_filter_.update((error - previous_error_) / dt);
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