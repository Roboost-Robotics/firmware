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
 * TODO: add anti-windup, gain scheduling, and output max/min
 *
 */
class PIDController
{
public:
    /**
     * @brief Construct a new PIDController object
     *
     * @param kp The proportional gain.
     * @param ki The integral gain.
     * @param kd The derivative gain.
     * @param max_expected_sampling_time The maximum expected sampling time.
     */
    PIDController(double kp, double ki, double kd,
                  double max_expected_sampling_time);

    /**
     * @brief Update the controller.
     *
     * @param setpoint The setpoint.
     * @param input The input value.
     * @return double The output value.
     */
    double update(double setpoint, double input);

private:
    double kp_;
    double ki_;
    double kd_;
    double max_expected_sampling_time_;
    double integral_;
    double previous_error_;
    LowPassFilter derivative_filter_;
    unsigned long last_update_time_;
};

#endif // CONTROLLERS_H