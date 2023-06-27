#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "motor_control/motor_drivers/motor_driver.hpp"

class MotorController {
public:
  MotorController(MotorDriver& motor_driver) : motor_driver_(motor_driver) {}
  
  virtual void set_rotation_speed(float desired_rotation_speed) = 0;

protected:
  MotorDriver& motor_driver_;
};

#endif // MOTOR_CONTROLLER_H
