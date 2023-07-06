/**
 * @file motor_controller.hpp //TODO
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "motor_control/motor_drivers/motor_driver.hpp"

/**
 * @brief //TODO
 * 
 */
class MotorController {
public:
  /**
   * @brief Construct a new Motor Controller object //TODO
   * 
   * @param motor_driver 
   */
  MotorController(MotorDriver& motor_driver) : motor_driver_(motor_driver) {}
  
  /**
   * @brief Destroy the Motor Controller object //TODO
   * 
   */
  virtual ~MotorController() {}

  /**
   * @brief Set the rotation speed object //TODO
   * 
   * @param desired_rotation_speed 
   */
  virtual void set_rotation_speed(float desired_rotation_speed) = 0;

protected:
  MotorDriver& motor_driver_;
};

#endif // MOTOR_CONTROLLER_H
