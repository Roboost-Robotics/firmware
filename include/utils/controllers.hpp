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
 * @brief PID controller parameters.
 *
 */
struct PIDControllerParameters
{
    double kp;
    double ki;
    double kd;
    double max_expected_sampling_time;
};

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
     * @param parameters The parameters of the PID controller.
     */
    PIDController(const PIDControllerParameters& parameters);

    /**
     * @brief Update the controller.
     *
     * @param setpoint The setpoint.
     * @param input The input value.
     * @return double The output value.
     */
    double update(double setpoint, double input);

private:
    const PIDControllerParameters& parameters_;
    double integral_;
    double previous_error_;
    LowPassFilter derivative_filter_;
    unsigned long last_update_time_;
};

#endif // CONTROLLERS_H