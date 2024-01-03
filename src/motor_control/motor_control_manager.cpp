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

#include "motor-control/motor_control_manager.hpp"

MotorControllerManager::MotorControllerManager(
    std::initializer_list<MotorController*> motor_controllers)
{
    for (MotorController* motor_controller : motor_controllers)
    {
        motor_controllers_.push_back({motor_controller, 0.0f});
    }
}

MotorControllerManager::~MotorControllerManager()
{
    for (std::pair<MotorController*, float>& pair : motor_controllers_)
    {
        delete pair.first;
    }
}

void MotorControllerManager::set_motor_speed(const uint8_t motor_index,
                                             float desired_speed)
{
    if (motor_index < 0 || motor_index >= motor_controllers_.size())
    {
        Serial.println("Invalid motor index");
    }
    else
    {
        motor_controllers_[motor_index].second = desired_speed;
    }
}

void MotorControllerManager::set_all_motor_speeds(const float desired_speed)
{
    for (std::pair<MotorController*, float>& pair : motor_controllers_)
    {
        pair.second = desired_speed;
    }
}

float MotorControllerManager::get_motor_speed(const uint8_t motor_index) const
{
    if (motor_index < 0 || motor_index >= motor_controllers_.size())
    {
        Serial.println("Invalid motor index");
        return 0.0f;
    }
    else
    {
        return motor_controllers_[motor_index].first->get_rotation_speed();
    }
}

uint8_t MotorControllerManager::get_motor_count() const
{
    return motor_controllers_.size();
}

void MotorControllerManager::update()
{
    int i = 0;
    for (std::pair<MotorController*, float>& pair : motor_controllers_)
    {
        // Serial.print(">motor ");
        // Serial.print(i);
        // Serial.print(" setpoint:");
        // Serial.println(pair.second);
        // Serial.print(">motor ");
        // Serial.print(i);
        // Serial.print(" measured:");
        // Serial.println(pair.first->get_rotation_speed());

        pair.first->set_rotation_speed(pair.second);
        i++;
    }
}
