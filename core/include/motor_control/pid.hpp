#ifndef PID_H
#define PID_H

class PIDController {
public:
  PIDController(float kp, float ki, float kd);

  float calculate(float setpoint, float actual_value);

private:
  float kp_, ki_, kd_;
  float integral_, previous_error_;
};

#endif // PID_H