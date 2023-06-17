#ifndef L298N_MOTOR_DRIVER_H
#define L298N_MOTOR_DRIVER_H

#include "motor_control/motor_drivers/motor_driver.hpp"

class L298NMotorDriver : public MotorDriver {
public:
  L298NMotorDriver(int pinIN1, int pinIN2, int pinENA);

  virtual void setPWM(int pwm) override;

  // Other L298N-specific functions...

private:
  int pinIN1_, pinIN2_, pinENA_;
  // Other L298N-specific variables...
};

#endif // L298N_MOTOR_DRIVER_H