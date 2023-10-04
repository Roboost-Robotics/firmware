#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <ArduinoEigen.h>

#include "kinematics/kinematics.hpp"
#include "motor-control/motor_control_manager.hpp"

/**
 * @brief The RobotController class manages the control of a robot's motors and
 *        implements odometry calculations based on its kinematics model.
 */
class RobotController
{
public:
    /**
     * @brief Construct a new Robot Controller object.
     *
     * @param motor_manager The motor control manager responsible for motor
     *                      control.
     * @param kinematics_model The kinematics model used for odometry
     * calculations.
     */
    RobotController(MotorControllerManager& motor_manager, Kinematics* kinematics_model);

    /**
     * @brief Update the robot's control loop. This method should be called
     *        periodically to control the robot's motors and update odometry.
     */
    void update();

    /**
     * @brief Get the current odometry estimation of the robot's state.
     *
     * @return Eigen::Vector<double, 6> A vector representing the robot's
     *         odometry, containing position (x, y, z) and orientation (roll,
     *         pitch, yaw) information.
     */
    Eigen::Vector<double, 6> get_odometry();

    /**
     * @brief Set the latest command for the robot's motion control.
     *
     * @param latest_command A vector containing the latest command for the
     * robot's motion control, currently only representing linear velocities
     * (vx, vy, vz).
     */
    void set_latest_command(const Eigen::Vector3d& latest_command);

private:
    MotorControllerManager& motor_manager_; // Reference to the motor control manager.
    Kinematics* kinematics_model_;          // Pointer to the kinematics model.

    Eigen::Vector3d latest_command_;    // Latest motion control command.
    Eigen::Vector<double, 6> odometry_; // Current odometry estimation.
};

#endif // ROBOTCONTROLLER_H
