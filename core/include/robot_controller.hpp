#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <BasicLinearAlgebra.h>

#include "kinematics/kinematics.hpp"
#include "motor_control/motor_control_manager.hpp"

/**
 * @brief //todo
 *
 */
class RobotController {
  public:
    /**
     * @brief Construct a new Robot Controller object //todo
     *
     * @param motor_manager
     * @param kinematics_model
     */
    RobotController(MotorControllerManager &motor_manager, Kinematics &kinematics_model);

    /**
     * @brief //todo
     *
     */
    void update();

    /**
     * @brief Get the odometry object //todo
     *
     * @return BLA::Matrix<6>
     */
    BLA::Matrix<6> get_odometry();

    /**
     * @brief Set the latest command object //todo
     *
     * @param latest_command
     */
    void set_latest_command(const BLA::Matrix<3> &latest_command);

  private:
    MotorControllerManager &motor_manager_;
    Kinematics &kinematics_model_;

    BLA::Matrix<3> latest_command_;
    BLA::Matrix<6> odometry_;
};

#endif   // ROBOTCONTROLLER_H
