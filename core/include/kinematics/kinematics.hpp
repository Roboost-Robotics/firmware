/**
 * @file kinematics.hpp
 * @author your name (you@domain.com) //todo
 * @brief
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <BasicLinearAlgebra.h>

#ifndef KINEMATICS_H
#define KINEMATICS_H

/**
 * @brief //todo
 *
 */
class Kinematics {
  public:
    /**
     * @brief //todo
     *
     * @param wheel_velocity
     * @return BLA::Matrix<3>
     */
    virtual BLA::Matrix<3> calculate_robot_velocity(const BLA::Matrix<4> &wheel_velocity) = 0;
    /**
     * @brief //todo
     *
     * @param robot_velocity
     * @return BLA::Matrix<4>
     */
    virtual BLA::Matrix<4> calculate_wheel_velocity(const BLA::Matrix<3> &robot_velocity) = 0;
};

/**
 * @brief //todo
 *
 */
class MecanumKinematics4W : public Kinematics {
  public:
    /**
     * @brief Construct a new Mecanum Kinematics 4 W object //todo
     *
     * @param wheel_radius
     * @param wheel_base
     * @param track_width
     */
    MecanumKinematics4W(double wheel_radius, double wheel_base, double track_width);

    /**
     * @brief //todo
     *
     * @param wheel_velocity
     * @return BLA::Matrix<3>
     */
    BLA::Matrix<3> calculate_robot_velocity(const BLA::Matrix<4> &wheel_velocity) override;
    /**
     * @brief //todo
     *
     * @param robot_velocity
     * @return BLA::Matrix<4>
     */
    BLA::Matrix<4> calculate_wheel_velocity(const BLA::Matrix<3> &robot_velocity) override;

  private:
    double wheel_radius_;   // radius of wheels
    double wheel_base_;     // distance between wheel contact point in x direction
    double track_width_;    // distance between wheel contact point in y direction
};

// todo add SwerveKinematics3W implementation

#endif   // KINEMATICS_H
