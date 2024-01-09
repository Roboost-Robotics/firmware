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
#include "utils/controllers.hpp"

/**
 * @brief Implementation of MotorController, which sets the control output
 * directy to the motor driver with encoder feedback and PID control.
 *
 */
class PIDMotorController : public MotorController
{
public:
    /**
     * @brief Construct a new PIDMotorController object
     *
     * @param motor_driver The motor driver to control.
     * @param encoder The encoder to read the rotation speed from.
     * @param pid_controller The PID controller to use.
     * @param input_filter The filter to use for the input.
     * @param output_filter The filter to use for the output.
     */
    PIDMotorController(MotorDriver& motor_driver, Encoder& encoder,
                       PIDController& pid_controller, Filter& input_filter,
                       Filter& output_filter, double min_output);

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
    Encoder& encoder_;
    PIDController& pid_;
    Filter& input_filter_;
    Filter& output_filter_;
    double min_output_;
};

#endif // PID_MOTOR_CONTROLLER_H
