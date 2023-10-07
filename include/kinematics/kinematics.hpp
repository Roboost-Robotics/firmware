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

#include <ArduinoEigen.h>

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
     * @return Eigen::Vector3d The calculated robot velocity.
     */
    virtual Eigen::Vector3d
    calculate_robot_velocity(const Eigen::VectorXd& wheel_velocity) = 0;

    /**
     * @brief Calculate wheel velocities based on robot velocity.
     *
     * @param robot_velocity The velocity of the robot.
     * @return Eigen::VectorXd The calculated wheel velocities.
     */
    virtual Eigen::VectorXd
    calculate_wheel_velocity(const Eigen::Vector3d& robot_velocity) = 0;
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
    MecanumKinematics4W(const float& wheel_radius, const float& wheel_base,
                        const float& track_width);

    /**
     * @brief Calculate robot velocity based on wheel velocities.
     *
     * @param wheel_velocity The velocities of individual wheels.
     * @return Eigen::Vector3d The calculated robot velocity.
     */
    Eigen::Vector3d
    calculate_robot_velocity(const Eigen::VectorXd& wheel_velocity) override;

    /**
     * @brief Calculate wheel velocities based on robot velocity.
     *
     * @param robot_velocity The velocity of the robot.
     * @return Eigen::VectorXd The calculated wheel velocities.
     */
    Eigen::VectorXd
    calculate_wheel_velocity(const Eigen::Vector3d& robot_velocity) override;

private:
    const float wheel_radius_; // Radius of the wheels.
    const float wheel_base_;   // Distance between wheel contact points in the x
                               // direction.
    const float track_width_;  // Distance between wheel contact points in the y
                               // direction.

    Eigen::Matrix<double, 4, 3> forward_kinematics_;
    Eigen::Matrix<double, 3, 4> inverse_kinematics_;
};

// Add more kinematics definitions here

#endif // KINEMATICS_H
