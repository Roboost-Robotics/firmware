/**
 * @file motor_driver.hpp //todo
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

/**
 * @brief //todo
 *
 */
class MotorDriver
{
public:
    /**
     * @brief Set the motor control object //todo
     *
     * @param control_value
     */
    virtual void set_motor_control(
        float
            control_value) = 0; // = 0 makes this function pure virtual, meaning
                                // it MUST be implemented by any derived class

    /**
     * @brief Destroy the Motor Driver object //todo
     *
     */
    virtual ~MotorDriver() {} // virtual destructor
};

#endif // MOTOR_DRIVER_H