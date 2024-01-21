/**
 * @file l298n_motor_driver.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief This file contains the implementation of the L298N motor driver.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "motor-control/motor-drivers/l298n_motor_driver.hpp"
#include "conf_hardware.h"
#include <Arduino.h>

L298NMotorDriver::L298NMotorDriver(const uint8_t& pin_in1,
                                   const uint8_t& pin_in2,
                                   const uint8_t& pin_ena,
                                   const uint8_t& pwm_channel)
    : pin_in1_(pin_in1), pin_in2_(pin_in2), pin_ena_(pin_ena),
      pwm_channel_(pwm_channel)
{
    // Initialize L298N...
    // setting pin modes
    pinMode(pin_in1_, OUTPUT);
    pinMode(pin_in2_, OUTPUT);
    pinMode(pin_ena_, OUTPUT);

    // configure PWM functionalities
    ledcSetup(pwm_channel_, M_PWM_FRQ, M_PWM_RES);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(pin_ena_, pwm_channel_);
}

void L298NMotorDriver::set_motor_control(float control_value)
{
    // control_value should be between -1 and 1
    control_value = (control_value < -1.0) ? -1.0 : control_value;
    control_value = (control_value > 1.0) ? 1.0 : control_value;

    // Set direction for L298N...
    bool direction = control_value >= 0;
    digitalWrite(pin_in1_, direction ? HIGH : LOW);
    digitalWrite(pin_in2_, direction ? LOW : HIGH);

    // Set PWM for L298N...
    u_int8_t pwm = static_cast<int>(std::abs(control_value) * 255);
    ledcWrite(pwm_channel_, pwm);
}