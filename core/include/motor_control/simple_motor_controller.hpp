#ifndef SIMPLE_MOTOR_CONTROLLER_H
#define SIMPLE_MOTOR_CONTROLLER_H

#include "motor_controller.hpp"

class SimpleMotorController : public MotorController {
public:
  SimpleMotorController(MotorDriver& motor_driver, float max_rotation_speed);

  void set_rotation_speed(float desired_rotation_speed) override;

private:
  float max_rotation_speed_;
};

#endif // SIMPLE_MOTOR_CONTROLLER_H
