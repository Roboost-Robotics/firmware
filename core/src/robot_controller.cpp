#include "robot_controller.hpp"

RobotController::RobotController(MotorControllerManager& motor_manager, Kinematics& kinematics_model)
: motor_manager_(motor_manager), kinematics_model_(kinematics_model) {
    // Initialize latestCommand and latestOdometry to default values here
    latest_command_ = BLA::Zeros<3>();
    odometry_ = BLA::Zeros<6>();
}

void RobotController::update() {
    // Use the KinematicsModel to calculate desired wheel speeds
    BLA::Matrix<4> desired_wheel_speeds = kinematics_model_.calculate_wheel_velocity(latest_command_);

    // Check if there are enough motor controllers in the manager
    if (motor_manager_.get_motor_count() < 4) {
        // Handle error: not enough motor controllers
        return;
    }

    // Use the MotorControllerManager to set these speeds
    for(int i = 0; i < 4; ++i) {
        motor_manager_.set_motor_speed(i, desired_wheel_speeds(i));
    }
}



BLA::Matrix<6> RobotController::get_odometry() {
    // Return the latest odometry data
    return odometry_;
}

void RobotController::set_latest_command(const BLA::Matrix<3>& latest_command) {
    latest_command_ = latest_command;
}
