#include "motor_control/motor_drivers/l298n_motor_driver.hpp"
#include <Arduino.h>

L298NMotorDriver::L298NMotorDriver(unsigned int pin_in1, unsigned int pin_in2, unsigned int pin_ena, unsigned int pwm_channel)
  : pin_in1_(pin_in1), pin_in2_(pin_in2), pin_ena_(pin_ena), pwm_channel_(pwm_channel) {
  // Initialize L298N...
  // setting PWM properties
  const int freq = 5000;
  const int resolution = 8;

  // configure PWM functionalities
  ledcSetup(pwm_channel_, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pin_ena_, pwm_channel_);
}

void L298NMotorDriver::set_motor_control(float control_value) {
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