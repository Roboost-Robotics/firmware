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
 * @brief Motor controller with encoder feedback and PID // TODO: a lot
 *
 */
class PIDMotorController : public MotorController
{
public:
    /**
     * @brief Construct a new Simple Motor Controller object
     *
     * @param motor_driver Motor driver to be used
     * @param max_rotation_speed Max rotational speed motor driver can output in
     * rad/sec
     */
    PIDMotorController(MotorDriver& motor_driver, Encoder& encoder);

    /**
     * @brief Set the rotation speed of the motor
     *
     * @param desired_rotation_speed desired rotation speed in rad/sec
     */
    void set_rotation_speed(float desired_rotation_speed);

private:
    Encoder& encoder_;
    PID pid_;

    // input is the measured rotation speed of the motor, output is the control signal,
    // setpoint is the desired rotation speed
    double input_, output_, setpoint_;

    double kp_ = 0.5, ki_ = 0.1, kd_ = 0.0;
};

#endif // PID_MOTOR_CONTROLLER_H
