#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <BasicLinearAlgebra.h>

#include "motor_control/motor_control_manager.hpp"
#include "kinematics/kinematics.hpp"

class RobotController {
public:
    RobotController(MotorControllerManager& motor_manager, Kinematics& kinematics_model);

    void update();
    
    BLA::Matrix<6> get_odometry();

    void set_latest_command(const BLA::Matrix<3>& latest_command);

private:
    MotorControllerManager& motor_manager_;
    Kinematics& kinematics_model_;

    BLA::Matrix<3> latest_command_;
    BLA::Matrix<6> odometry_;
};

#endif //ROBOTCONTROLLER_H
