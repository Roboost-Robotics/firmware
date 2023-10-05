/**
 * @file pid_motor_controller.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Implementation of MotorController, which sets the control output
 * directy to the motor driver with encoder feedback and PID control.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PID_MOTOR_CONTROLLER_H
#define PID_MOTOR_CONTROLLER_H

#include "motor-control/encoder.hpp"
#include "motor_controller.hpp"
#include <PID_v1.h>
#include <deque>

/**
 * @brief Implementation of MotorController, which sets the control output
 * directy to the motor driver with encoder feedback and PID control.
 *
 * TODO: Model Predictive Control
 */
class PIDMotorController : public MotorController
{
public:
    /**
     * @brief Construct a new PIDMotorController object
     *
     * @param motor_driver The motor driver to be controlled.
     * @param encoder The encoder to be used for feedback.
     * @param kp The proportional gain of the PID controller.
     * @param ki The integral gain of the PID controller.
     * @param kd The derivative gain of the PID controller.
     * @param control_upper_limit The upper limit of the control output.
     * @param control_lower_limit The lower limit of the control output.
     *
     * @note double kp = 0.1, double ki = 0.8, double kd = 0.001 are good for no
     * load
     */
    PIDMotorController(MotorDriver& motor_driver, Encoder& encoder,
                       double kp = 0.17, double ki = 0.0, double kd = 0.0,
                       const double control_upper_limit = 1.0,
                       const double control_lower_limit = -1.0);

    /**
     * @brief Set the rotation speed of the motor.
     *
     * @param desired_rotation_speed The desired rotation speed in rad/s.
     */
    void set_rotation_speed(float desired_rotation_speed);

    /**
     * @brief Get the rotation speed of the motor.
     *
     * @return float The rotation speed in rad/s.
     */
    float get_rotation_speed();

private:
    /**
     * @brief Filter the output to reduce noise.
     *
     * @param output The output to be filtered.
     * @return double The filtered output term.
     */
    double filter_output(double output);

    /**
     * @brief Filter the input to reduce noise.
     *
     * @param input The input to be filtered.
     * @return double The filtered input term.
     */
    double filter_input(double input);

    /**
     * @brief Apply anti-windup to the integral term.
     *
     * @param control The control output for limiting the integral term.
     */
    void apply_anti_windup(float control);

    /**
     * @brief Update the PID gains based on error.
     *
     */
    void gain_scheduling();

    Encoder& encoder_;
    PID pid_;

    std::deque<double> output_history_;
    std::deque<double> input_history_;
    const int filter_window_size_ = 4;

    const double control_upper_limit_;
    const double control_lower_limit_;

    double input_, output_, setpoint_;
    double kp_, ki_, kd_;
};

#endif // PID_MOTOR_CONTROLLER_H
