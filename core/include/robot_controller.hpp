#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <BasicLinearAlgebra.h>

#include "motor_control/motor_control_manager.hpp"
#include "kinematics/kinematics.hpp"

/**
 * @brief //TODO
 * 
 */
class RobotController {
public:
/**
 * @brief Construct a new Robot Controller object //TODO
 * 
 * @param motor_manager 
 * @param kinematics_model 
 */
    RobotController(MotorControllerManager& motor_manager, Kinematics& kinematics_model);

/**
 * @brief //TODO
 * 
 */
    void update();
    
/**
 * @brief Get the odometry object //TODO
 * 
 * @return BLA::Matrix<6> 
 */
    BLA::Matrix<6> get_odometry();

/**
 * @brief Set the latest command object //TODO
 * 
 * @param latest_command 
 */
    void set_latest_command(const BLA::Matrix<3>& latest_command);

private:
    MotorControllerManager& motor_manager_;
    Kinematics& kinematics_model_;

    BLA::Matrix<3> latest_command_;
    BLA::Matrix<6> odometry_;
};

#endif //ROBOTCONTROLLER_H
