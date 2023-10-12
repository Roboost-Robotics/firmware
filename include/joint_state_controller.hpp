#ifndef JOINTSTATECONTROLLER_H
#define JOINTSTATECONTROLLER_H

#ifdef VELOCITYCONTROLLER_H
#error                                                                         \
    "joint_state_controller.hpp and velocity_controller.hpp cannot be used at the same time!"
#endif

#include <ArduinoEigen.h>

#include "motor-control/motor_control_manager.hpp"
#include <sensor_msgs/msg/joint_state.h>

/**
 * @brief The JointStateController class manages the control of a robot's motors
 * based on the joint state messages received from the ROS2 network.
 */
class JointStateController
{
public:
    /**
     * @brief Construct a new Joint State Controller object.
     *
     * @param motor_manager The motor control manager responsible for motor
     *                      control.
     */
    JointStateController(MotorControllerManager& motor_manager);

    /**
     * @brief Update the robot's control loop. This method should be called
     *        periodically to update the robot's motor control.
     */
    void update();

    /**
     * @brief Set the latest command for the robot's motion control.
     *
     * @param latest_command A vector containing the latest command for each
     *                      motor.
     */
    void set_latest_command(const Eigen::VectorXd& latest_command);

    /**
     * @brief Get the latest joint state message.
     *
     * @return sensor_msgs__msg__JointState The latest joint state message.
     */
    sensor_msgs__msg__JointState get_joint_state_msg();

private:
    MotorControllerManager& motor_manager_;

    Eigen::VectorXd latest_command_;
    sensor_msgs__msg__JointState joint_state_msg_;
};

#endif // JOINTSTATECONTROLLER_H
