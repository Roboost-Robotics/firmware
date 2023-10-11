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

PIDController::PIDController(const PIDControllerParameters& parameters)
    : parameters_(parameters), integral_(0.0), previous_error_(0.0),
      derivative_filter_(1.0 /
                             (1.0 + 2.0 * PI * parameters.kd *
                                        parameters.max_expected_sampling_time),
                         parameters.max_expected_sampling_time)
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

    double output = parameters_.kp * error + parameters_.ki * integral_ +
                    parameters_.kd * derivative;

    // Serial.print(">PIDController input:");
    // Serial.println(input);
    // Serial.print(">PIDController output:");
    // Serial.println(output);
    // Serial.print(">PIDController setpoint:");
    // Serial.println(setpoint);
    // Serial.print(">PIDController error:");
    // Serial.println(error);
    // Serial.print(">PIDController integral:");
    // Serial.println(integral_);
    // Serial.print(">PIDController derivative:");
    // Serial.println(derivative);
    // Serial.print(">PIDController sampling_time:");
    // Serial.println(sampling_time);
    // Serial.print(">PIDController kp:");
    // Serial.println(parameters_.kp);
    // Serial.print(">PIDController ki:");
    // Serial.println(parameters_.ki);
    // Serial.print(">PIDController kd:");
    // Serial.println(parameters_.kd);
    // Serial.println(">PIDController max_expected_sampling_time:");
    // Serial.println(parameters_.max_expected_sampling_time);

    return output;
}