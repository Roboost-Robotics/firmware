#include <BasicLinearAlgebra.h>
#include "conf_robot.h"

#ifndef KINEMATICS_H
#define KINEMATICS_H

// Function to calculate wheel velocities
BLA::Matrix<4> calculateWheelVelocity(BLA::Matrix<3> robotVelocity);
#if defined(MECANUM_4WHEEL)
  /**
   * @brief Calculates wheel velocity based on given robot velocity
   * 
   * @param robotVelocity 
   * @return BLA::Matrix<4> 
   */
  BLA::Matrix<4> calculateWheelVelocity(BLA::Matrix<3> robotVelocity);

  /**
   * @brief Calculates the velocity in direction x and y, as well as the angular velocity around the z axis [m/s] [m/s] [rad/s].
   * 
   * @param wheelVelocity 
   * @return BLA::Matrix<3> velocity of the robot in x, y and rotational velocity around z
   */
  BLA::Matrix<3> calculateRobotVelocity(BLA::Matrix<4> wheelVelocity);
#elif defined(SWERVE_3WHEEL)
    // TODO: Calculate forward and reverse kinematics model
#else
    // TODO: throw compile error
#endif

#endif // KINEMATICS_H