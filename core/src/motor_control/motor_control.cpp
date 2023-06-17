#include "motor_control/motor_control.hpp"

MotorController::MotorController(PIDController& pid_controller, Encoder& encoder, MotorDriver& motor_driver)
  : pid_controller_(pid_controller), encoder_(encoder), motor_driver_(motor_driver) {
}

void MotorController::setVelocity(float desired_velocity) {
  float actual_velocity = encoder_.readVelocity();
  float control_signal = pid_controller_.calculate(desired_velocity, actual_velocity);
  motor_driver_.setPWM(control_signal);
}

// Other motor-related function implementations...
