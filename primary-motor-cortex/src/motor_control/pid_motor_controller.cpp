/**
 * @file pid_motor_controller.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief This file contains the implementation of a PID motor controller.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "motor-control/pid_motor_controller.hpp"
#include <Arduino.h>

PIDMotorController::PIDMotorController(MotorDriver& motor_driver, Encoder& encoder)
    : MotorController(motor_driver), encoder_(encoder), pid_(&input_, &output_, &setpoint_, kp_, ki_, kd_, DIRECT)
{
    pid_.SetMode(AUTOMATIC);
    pid_.SetTunings(kp_, ki_, kd_);
    pid_.SetOutputLimits(-1, 1);
}

void PIDMotorController::set_rotation_speed(float desired_rotation_speed)
{
    encoder_.update();
    setpoint_ = desired_rotation_speed;
    input_ = encoder_.get_velocity();

    bool output_computed = pid_.Compute();

    motor_driver_.set_motor_control(output_);

    Serial.print("kp: ");
    Serial.print(pid_.GetKp());
    Serial.print(" ki: ");
    Serial.print(pid_.GetKi());
    Serial.print(" kd: ");
    Serial.print(pid_.GetKd());
    Serial.print(" computed: ");
    Serial.print(output_computed);
    Serial.print(" input: ");
    Serial.print(input_);
    Serial.print(" output: ");
    Serial.print(output_);
    Serial.print(" setpoint: ");
    Serial.println(setpoint_);
}
