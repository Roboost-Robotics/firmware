#ifndef L298N_MOTOR_DRIVER_H
#define L298N_MOTOR_DRIVER_H

#include "motor_control/motor_drivers/motor_driver.hpp"

class L298NMotorDriver : public MotorDriver {
public:
  L298NMotorDriver(unsigned int pin_in1, unsigned int pin_in2, unsigned int pin_ena, unsigned int pwm_channel);

  void set_motor_control(float control_value);

private:
  const unsigned int pin_in1_;
  const unsigned int pin_in2_;
  const unsigned int pin_ena_;
  const unsigned int pwm_channel_;
};

#endif // L298N_MOTOR_DRIVER_H