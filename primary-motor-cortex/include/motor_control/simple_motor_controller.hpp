/**
 * @file simple_motor_controller.hpp //todo
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef SIMPLE_MOTOR_CONTROLLER_H
#define SIMPLE_MOTOR_CONTROLLER_H

#include "motor_controller.hpp"

/**
 * @brief Motor controller without encoder feedback or PID
 *
 */
class SimpleMotorController : public MotorController
{
public:
    /**
     * @brief Construct a new Simple Motor Controller object
     *
     * @param motor_driver Motor driver to be used
     * @param max_rotation_speed Max rotational speed motor driver can output in
     * rad/sec
     */
    SimpleMotorController(MotorDriver& motor_driver, float max_rotation_speed);

    /**
     * @brief Set the rotation speed of the motor
     *
     * @param desired_rotation_speed desired rotation speed in rad/sec
     */
    void set_rotation_speed(float desired_rotation_speed);

private:
    float max_rotation_speed_;
};

#endif // SIMPLE_MOTOR_CONTROLLER_H
