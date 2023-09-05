/**
 * @file motor_control_manager.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief A manager for controlling motors and managing their speeds.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef MOTOR_CONTROLLER_MANAGER_H
#define MOTOR_CONTROLLER_MANAGER_H

#include "motor-control/motor_controller.hpp"
#include <ArduinoEigen.h>
#include <vector>

/**
 * @brief The MotorControllerManager class manages a collection of motors and
 *        their desired speeds.
 */
class MotorControllerManager
{
public:
    /**
     * @brief Construct a new Motor Controller Manager object.
     *
     * @param motor_controllers An initializer list of MotorController pointers.
     */
    MotorControllerManager(
        std::initializer_list<MotorController*> motor_controllers);

    /**
     * @brief Set the desired speed for a specific motor.
     *
     * @param motor_index The index of the motor.
     * @param desired_speed The desired speed value.
     */
    void set_motor_speed(int motor_index, float desired_speed);

    /**
     * @brief Set the desired speed for all motors.
     *
     * @param desired_speed The desired speed value.
     */
    void set_all_motor_speeds(float desired_speed);

    /**
     * @brief Get the desired speed of a specific motor.
     *
     * @param motor_index The index of the motor.
     * @return float The desired speed value.
     */
    float get_motor_speed(int motor_index) const;

    /**
     * @brief Get the number of MotorControllers in the manager.
     *
     * @return int The number of motors.
     */
    int get_motor_count() const;

    /**
     * @brief Update the MotorControllers to set the new desired rotational
     * speed.
     */
    void update();

    /**
     * @brief Destroy the Motor Controller Manager object and free up the
     * memory.
     */
    ~MotorControllerManager();

private:
    std::vector<std::pair<MotorController*, float>>
        motor_controllers_; // Vector to hold MotorController pointers and
                            // desired speeds.
};

#endif // MOTOR_CONTROLLER_MANAGER_H
