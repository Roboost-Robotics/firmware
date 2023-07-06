/**
 * @file pid.hpp //TODO
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef PID_H
#define PID_H

/**
 * @brief //TODO
 * 
 */
class PIDController {
public:
  /**
   * @brief Construct a new PIDController object //TODO
   * 
   * @param kp 
   * @param ki 
   * @param kd 
   */
  PIDController(float kp, float ki, float kd);

  /**
   * @brief //TODO
   * 
   * @param setpoint 
   * @param actual_value 
   * @return float 
   */
  float calculate(float setpoint, float actual_value);

private:
  float kp_, ki_, kd_;
  float integral_, previous_error_;
};

#endif // PID_H