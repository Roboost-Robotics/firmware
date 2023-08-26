/**
 * @file kinematics.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Kinematics base class definitions.
 * @version 1.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <BasicLinearAlgebra.h>

#ifndef KINEMATICS_H
#define KINEMATICS_H

/**
 * @brief Abstract base class for defining kinematics calculations.
 *
 * This class provides an interface for calculating robot velocity and wheel
 * velocity based on the kinematic properties of the robot.
 */
class Kinematics
{
public:
    /**
     * @brief Calculate robot velocity based on wheel velocities.
     *
     * @param wheel_velocity The velocities of individual wheels.
     * @return BLA::Matrix<3> The calculated robot velocity.
     */
    virtual BLA::Matrix<3>
    calculate_robot_velocity(const BLA::Matrix<4>& wheel_velocity) = 0;

    /**
     * @brief Calculate wheel velocities based on robot velocity.
     *
     * @param robot_velocity The velocity of the robot.
     * @return BLA::Matrix<4> The calculated wheel velocities.
     */
    virtual BLA::Matrix<4>
    calculate_wheel_velocity(const BLA::Matrix<3>& robot_velocity) = 0;
};

/**
 * @brief Mecanum kinematics for a 4-wheel robot.
 *
 * This class implements the kinematics calculations for a 4-wheel mecanum drive
 * robot. It takes into account the wheel radius, wheel base, and track width of
 * the robot.
 */
class MecanumKinematics4W : public Kinematics
{
public:
    /**
     * @brief Construct a new Mecanum Kinematics 4W object with given
     * parameters.
     *
     * @param wheel_radius The radius of the wheels.
     * @param wheel_base The distance between wheel contact points in the x
     * direction.
     * @param track_width The distance between wheel contact points in the y
     * direction.
     */
    MecanumKinematics4W(double wheel_radius, double wheel_base,
                        double track_width);

    /**
     * @brief Calculate robot velocity based on wheel velocities.
     *
     * @param wheel_velocity The velocities of individual wheels.
     * @return BLA::Matrix<3> The calculated robot velocity.
     */
    BLA::Matrix<3>
    calculate_robot_velocity(const BLA::Matrix<4>& wheel_velocity) override;

    /**
     * @brief Calculate wheel velocities based on robot velocity.
     *
     * @param robot_velocity The velocity of the robot.
     * @return BLA::Matrix<4> The calculated wheel velocities.
     */
    BLA::Matrix<4>
    calculate_wheel_velocity(const BLA::Matrix<3>& robot_velocity) override;

private:
    double wheel_radius_; // Radius of the wheels.
    double wheel_base_;   // Distance between wheel contact points in the x
                          // direction.
    double track_width_;  // Distance between wheel contact points in the y
                          // direction.
};

// Add more kinematics definitions here

#endif // KINEMATICS_H"