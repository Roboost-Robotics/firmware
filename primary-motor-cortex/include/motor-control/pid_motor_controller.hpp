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
     * @param motor_driver The motor driver to be controlled.
     * @param encoder The encoder to be used for feedback.
     * @param kp The proportional gain of the PID controller.
     * @param ki The integral gain of the PID controller.
     * @param kd The derivative gain of the PID controller.
     */
    PIDMotorController(MotorDriver& motor_driver, Encoder& encoder, double kp = 0.1, double ki = 0.8,
                       double kd = 0.001);

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
    PID pid_;

    double input_, output_, setpoint_;
    double kp_, ki_, kd_;
};

#endif // PID_MOTOR_CONTROLLER_H
