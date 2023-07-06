/**
 * @file motor_control_manager.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "motor_control/motor_control_manager.hpp"

MotorControllerManager::MotorControllerManager(std::initializer_list<MotorController*> motor_controllers) {
    for (MotorController* motor_controller : motor_controllers) {
        motor_controllers_.push_back({motor_controller, 0.0f});
    }
}

MotorControllerManager::~MotorControllerManager() {
    for(auto& pair : motor_controllers_) {
        delete pair.first;
    }
}

void MotorControllerManager::set_motor_speed(int motor_index, float desired_speed) {
    if(motor_index < 0 || motor_index >= motor_controllers_.size()) {
        // Handle invalid index error here...
    } else {
        motor_controllers_[motor_index].second = desired_speed;
    }
}

void MotorControllerManager::set_all_motor_speeds(float desired_speed) {
    for(auto& pair : motor_controllers_) {
        pair.second = desired_speed;
    }
}

float MotorControllerManager::get_motor_speed(int motor_index) const {
    if(motor_index < 0 || motor_index >= motor_controllers_.size()) {
        // Handle invalid index error here...
        return 0.0f; // Or some other default value.
    } else {
        return motor_controllers_[motor_index].second;
    }
}

int MotorControllerManager::get_motor_count() const {
    return motor_controllers_.size();
}


void MotorControllerManager::update() {
    for(auto& pair : motor_controllers_) {
        pair.first->set_rotation_speed(pair.second);
    }
}
