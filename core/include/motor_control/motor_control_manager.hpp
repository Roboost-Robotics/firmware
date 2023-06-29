#ifndef MOTOR_CONTROLLER_MANAGER_H
#define MOTOR_CONTROLLER_MANAGER_H

#include "motor_control/motor_controller.hpp"
#include <BasicLinearAlgebra.h>
#include <vector>

class MotorControllerManager {
public:
    // Constructor that accepts a list of MotorControllers.
    MotorControllerManager(std::initializer_list<MotorController*> motor_controllers);

    // Set the desired speed for a specific motor.
    void set_motor_speed(int motor_index, float desired_speed);

    // Set the desired speed for all motors.
    void set_all_motor_speeds(float desired_speed);

    // Get the desired speed of a specific motor.
    float get_motor_speed(int motor_index) const;

    // Get the number of MotorControllers in the manager.
    int get_motor_count() const;

    // Update the MotorControllers to set the new desired rotational speed.
    void update();

    // Destructor to free up the memory of MotorControllers
    ~MotorControllerManager();

private:
    std::vector<std::pair<MotorController*, float>> motor_controllers_; // Vector to hold MotorController pointers and desired speeds.
};

#endif // MOTOR_CONTROLLER_MANAGER_H
