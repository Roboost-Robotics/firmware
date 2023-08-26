/**
 * @file pid.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Proportional-Integral-Derivative (PID) controller for versitile usage.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef PID_H
#define PID_H

/**
 * @class PIDController
 * @brief Implements a Proportional-Integral-Derivative (PID) controller for
 * feedback control systems.
 *
 * This class implements a PID controller, which is a common feedback control
 * algorithm used to regulate a system's output based on the difference between
 * a desired setpoint and the actual value. The controller calculates control
 * signals based on proportional, integral, and derivative terms.
 */
class PIDController
{
public:
    /**
     * @brief Constructor for creating a new PIDController object.
     *
     * @param kp The proportional gain coefficient.
     * @param ki The integral gain coefficient.
     * @param kd The derivative gain coefficient.
     */
    PIDController(float kp, float ki, float kd);

    /**
     * @brief Calculate the control signal using the PID algorithm.
     *
     * This method calculates the control signal using the
     * Proportional-Integral-Derivative (PID) algorithm. It takes a setpoint
     * value and an actual value as input and returns the control signal that
     * should be applied to the system.
     *
     * @param setpoint The desired setpoint value.
     * @param actual_value The actual value of the system.
     * @return The calculated control signal.
     */
    float calculate(float setpoint, float actual_value);

private:
    float kp_, ki_, kd_;
    float integral_, previous_error_;
};

#endif // PID_H