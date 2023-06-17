#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "motor_control/pid.hpp"
#include "motor_control/encoder.hpp"
#include "motor_control/motor_drivers/motor_driver.hpp"

class MotorController {
public:
  MotorController(PIDController& pid_controller, Encoder& encoder, MotorDriver& motor_driver);

  void setVelocity(float desired_velocity);

  // Other motor-related functions...

private:
  PIDController& pid_controller_;
  Encoder& encoder_;
  MotorDriver& motor_driver_;
  // Other motor-related variables...
};

#endif // MOTOR_CONTROLLER_H