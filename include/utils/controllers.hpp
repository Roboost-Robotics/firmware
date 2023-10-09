/**
 * @file controllers.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility functions and classes for controllers.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "utils/constants.h"
#include "utils/filters.hpp"
#include <Arduino.h>

/**
 * @brief PID controller class.
 *
 */
class PIDController
{
public:
    /**
     * @brief Construct a new PID Controller object
     *
     * @param kp The proportional gain.
     * @param ki The integral gain.
     * @param kd The derivative gain.
     * @param max_sampling_time The sampling time.
     */
    PIDController(double kp, double ki, double kd, double max_sampling_time);

    /**
     * @brief Update the controller.
     *
     * @param setpoint The setpoint.
     * @param input The input value.
     * @return double The output value.
     */
    double update(double setpoint, double input, double sampling_time);

private:
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double previous_error_;
    LowPassFilter derivative_filter_;
    LowPassFilter input_filter_;
};

PIDController::PIDController(double kp, double ki, double kd,
                             double max_sampling_time)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), previous_error_(0.0),
      derivative_filter_(1.0 / (1.0 + 2.0 * PI * kd * max_sampling_time),
                         max_sampling_time),
      input_filter_(1.0 /
                        (1.0 + 2.0 * PI * ki *
                                   max_sampling_time), // TODO: What should be
                                                       // the cutoff frequency?
                    max_sampling_time)
{
}

double PIDController::update(double setpoint, double input,
                             double sampling_time)
{
    Serial.print(">input:");
    Serial.println(input);
    input = input_filter_.update(input);

    Serial.print(">filtered input:");
    Serial.println(input);

    double error = setpoint - input;
    integral_ += error * sampling_time;
    double derivative =
        derivative_filter_.update((error - previous_error_) / sampling_time);
    previous_error_ = error;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    Serial.print(">setpoint:");
    Serial.println(setpoint);
    Serial.print(">error:");
    Serial.println(error);
    Serial.print(">integral:");
    Serial.println(integral_);
    Serial.print(">derivative:");
    Serial.println(derivative);
    Serial.print(">output:");
    Serial.println(output);

    return output;
}

#endif // CONTROLLERS_H