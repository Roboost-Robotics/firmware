# README

nices tutorial:
https://micro.ros.org/docs/tutorials/core/first_application_linux/





## TODO
- Implememt the use of URDF files
- Make virtual motor class that uses PID controll; implementations are specific like H-bridge or VESC
- Implement ROS communication

## Motorshield Firmware
### Overview
This program controls the movement of a four-wheeled robot using encoder feedback and PID control. It also includes functions for calculating the robot's velocity using forward and inverse kinematics models.

### Variables
- `count_BL`, `count_BR`, `count_FL`, `count_FR`: 16-bit integers representing the number of encoder counts for the back left, back right, front left, and front right wheels, respectively
- `rotationspeed_BL`, `rotationspeed_BR`, `rotationspeed_FL`, `rotationspeed_FR`: doubles representing the rotational speed of the back left, back right, front left, and front right wheels, respectively, in revolutions per second
- `dutyCycle_BL`, `dutyCycle_BR`, `dutyCycle_FL`, `dutyCycle_FR`: doubles representing the duty cycle of the back left, back right, front left, and front right motors, respectively, as a percentage of the maximum duty cycle
- `wantedWheelVel_BL`, `wantedWheelVel_BR`, `wantedWheelVel_FL`, `wantedWheelVel_FR`: doubles representing the desired rotational speed of the back left, back right, front left, and front right wheels, respectively, in revolutions per second

### Interrupt Routines
- `function_ISR_EC_BL()`: interrupt service routine for the back left encoder
- `function_ISR_EC_BR()`: interrupt service routine for the back right encoder
- `function_ISR_EC_FL()`: interrupt service routine for the front left encoder
- `function_ISR_EC_FR()`: interrupt service routine for the front right encoder

### PID Control
The program uses PID control to maintain the desired rotational speed of the wheels. The PID constants `Kp`, `Ki`, and `Kd` are set to 2, 1, and 1, respectively. The PID control is initialized for each wheel with the following lines of code:

```C++
PID PID_BL(&wantedWheelVel_BL, &dutyCycle_BL, &rotationspeed_BL, Kp, Ki, Kd, AUTOMATIC);
PID PID_BR(&wantedWheelVel_BR, &dutyCycle_BR, &rotationspeed_BR, Kp, Ki, Kd, AUTOMATIC);
PID PID_FL(&wantedWheelVel_FL, &dutyCycle_FL, &rotationspeed_FL, Kp, Ki, Kd, AUTOMATIC);
PID PID_FR(&wantedWheelVel_FR, &dutyCycle_FR, &rotationspeed_FR, Kp, Ki, Kd, AUTOMATIC);
```


## Kinematics Functions
- `calculateWheelVelocity(BLA::Matrix<3> robotVelocity)`: calculates the rotational speed of the wheels in revolutions per second based on the robot's velocity in the x, y, and z directions. The input `robotVelocity` is a 3x1 matrix with elements `[vx, vy, wz]`, where `vx` and `vy` are the velocities in the x and y directions, respectively, and `wz` is the angular velocity around the z axis, all in standard units. The function returns a 4x1 matrix with elements `[v_BL, v_BR, v_FL, v_FR]`, representing the rotational speed of the back left, back right, front left, and front right wheels, respectively.

- `calculateRobotVelocity(BLA::Matrix<4> wheelVelocity)`: calculates the robot's velocity in the x, y, and z directions based on the rotational speed of the wheels. The input `wheelVelocity` is a 4x1 matrix with elements `[v_BL, v_BR, v_FL, v_FR]`, representing the rotational speed of the back left, back right, front left, and front right wheels, respectively. The function returns a 3x1 matrix with elements `[vx, vy, wz]`, where `vx` and `vy` are the velocities in the x and y directions, respectively, and `wz` is the angular velocity around the z axis, all in standard units.

## Other Functions
`masterCommunicationRoutine(void* parameters)`: a routine for handling communication with the master controller
`motorHardwareSetup()`: a function for setting up the hardware for the motors
`motorHardwareLoop()`: a loop for updating the motor hardware based on the PID control output
Constants
`M_BL_PWM_CNL`, `M_BR_PWM_CNL`, `M_FL_PWM_CNL`, `M_FR_PWM_CNL`: constants representing the channel numbers for the back left, back right, front left, and front right motors, respectively
`M_PWM_FRQ`: constant representing the PWM frequency for the motors
`M_PWM_RES`: constant representing the PWM resolution for the motors
`L_X` and `L_Y`: constants representing the dimensions of the robot in the x and y directions, respectively
`WHEELRADIUS`: constant representing the radius of the wheels