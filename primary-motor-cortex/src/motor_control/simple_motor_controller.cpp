/**
 * @file simple_motor_controller.cpp //todo
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "motor-control/simple_motor_controller.hpp"
#include <Arduino.h>
#include <algorithm> // for std::clamp

SimpleMotorController::SimpleMotorController(MotorDriver& motor_driver,
                                             float max_rotation_speed)
    : MotorController(motor_driver), max_rotation_speed_(max_rotation_speed)
{
}

void SimpleMotorController::set_rotation_speed(float desired_rotation_speed)
{
    desired_rotation_speed = std::clamp(
        desired_rotation_speed, -max_rotation_speed_, max_rotation_speed_);

    // Map the desired_rotation_speed to a value between -1 and 1
    float control_value = desired_rotation_speed / max_rotation_speed_;

    motor_driver_.set_motor_control(control_value);

    // Print all values for debugging
    // Serial.print("Desired rotation speed: ");
    // Serial.println(desired_rotation_speed);
    // Serial.print("Control value: ");
    // Serial.println(control_value);
}
