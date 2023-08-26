/**
 * @file motor_driver.hpp
 * @author Defines the MotorDriver interface for controlling motors.
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
 * @class MotorDriver
 * @brief Abstract base class for motor control.
 *
 * This class defines an interface for controlling motors. Subclasses of
 * MotorDriver are expected to implement the set_motor_control method to provide
 * motor control functionality.
 */
class MotorDriver
{
public:
    /**
     * @brief Set the motor control value.
     *
     * This method allows setting the control value for the motor. The
     * interpretation of the control value may vary based on the specific motor
     * driver implementation.
     *
     * @param control_value The control value for the motor.
     */
    virtual void set_motor_control(float control_value) = 0;

    /**
     * @brief Destructor for the Motor Driver object.
     */
    virtual ~MotorDriver() {}
};

#endif // MOTOR_DRIVER_H