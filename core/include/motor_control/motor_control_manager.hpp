/**
 * @file motor_control_manager.hpp //todo
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef MOTOR_CONTROLLER_MANAGER_H
#define MOTOR_CONTROLLER_MANAGER_H

#include "motor_control/motor_controller.hpp"
#include <BasicLinearAlgebra.h>
#include <vector>

/**
 * @brief //todo
 *
 */
class MotorControllerManager {
  public:
    // Constructor that accepts a list of MotorControllers.
    /**
     * @brief Construct a new Motor Controller Manager object //todo
     *
     * @param motor_controllers
     */
    MotorControllerManager(std::initializer_list<MotorController *> motor_controllers);

    // Set the desired speed for a specific motor.
    /**
     * @brief Set the motor speed object //todo
     *
     * @param motor_index
     * @param desired_speed
     */
    void set_motor_speed(int motor_index, float desired_speed);

    // Set the desired speed for all motors.
    /**
     * @brief Set the all motor speeds object //todo
     *
     * @param desired_speed
     */
    void set_all_motor_speeds(float desired_speed);

    // Get the desired speed of a specific motor.
    /**
     * @brief Get the motor speed object //todo
     *
     * @param motor_index
     * @return float
     */
    float get_motor_speed(int motor_index) const;

    // Get the number of MotorControllers in the manager.
    /**
     * @brief Get the motor count object //todo
     *
     * @return int
     */
    int get_motor_count() const;

    // Update the MotorControllers to set the new desired rotational speed.
    /**
     * @brief //todo
     *
     */
    void update();

    // Destructor to free up the memory of MotorControllers
    /**
     * @brief Destroy the Motor Controller Manager object //todo
     *
     */
    ~MotorControllerManager();

  private:
    std::vector<std::pair<MotorController *, float>>
        motor_controllers_;   // Vector to hold MotorController pointers and desired speeds.
};

#endif   // MOTOR_CONTROLLER_MANAGER_H
