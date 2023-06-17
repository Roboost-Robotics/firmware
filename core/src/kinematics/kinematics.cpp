#include "kinematics/kinematics.hpp"

#if defined(MECANUM_4WHEEL)
  /**
   * @brief Calculates wheel velocity based on given robot velocity
   * 
   * @param robotVelocity 
   * @return BLA::Matrix<4> 
   */
  BLA::Matrix<4> calculateWheelVelocity(BLA::Matrix<3> robotVelocity){
    
    BLA::Matrix<4> wheelVelocity;
    BLA::Matrix<4, 3> forwardKinematicsModel = { 1, -1, -(L_X + L_Y),
                                                1, 1, L_X + L_Y,
                                                1, 1, -(L_X + L_Y),
                                                1, -1, L_X + L_Y};
    wheelVelocity = forwardKinematicsModel * robotVelocity;
    wheelVelocity *=  1 / WHEELRADIUS;

    return wheelVelocity;
  }

  /**
   * @brief Calculates the velocity in direction x and y, as well as the angular velocity around the z axis [m/s] [m/s] [rad/s].
   * 
   * @param wheelVelocity 
   * @return BLA::Matrix<3> velocity of the robot in x, y and rotational velocity around z
   */
  BLA::Matrix<3> calculateRobotVelocity(BLA::Matrix<4> wheelVelocity){
    BLA::Matrix<3> robotVelocity;

    BLA::Matrix<3, 4> inverseKinematicsModel = { 1, 1, 1, 1, 
                                                -1, 1, 1, -1, 
                                                -1/(L_X + L_Y), 1/(L_X + L_Y), -1/(L_X + L_Y), 1/(L_X + L_Y)};

    robotVelocity = inverseKinematicsModel * wheelVelocity;
    robotVelocity *= WHEELRADIUS / 4;

    return robotVelocity;
  }
#elif defined(SWERVE_3WHEEL)
    // TODO
#else
    // TODO: throw compile error
#endif