#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

class MotorDriver {
public:
  virtual void setPWM(int pwm) = 0; // = 0 makes this function pure virtual, meaning it MUST be implemented by any derived class
  virtual ~MotorDriver() {} // virtual destructor
};

#endif // MOTOR_DRIVER_H