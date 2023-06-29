#include "ros_handler.hpp"
#include "robot_controller.hpp"
#include "motor_control/simple_motor_controller.hpp"
#include "motor_control/motor_drivers/l298n_motor_driver.hpp"
#include "conf_robot.h"

L298NMotorDriver driver_M0(M0_IN1, M0_IN2, M0_ENA, M0_PWM_CNL);
L298NMotorDriver driver_M1(M1_IN1, M1_IN2, M1_ENA, M1_PWM_CNL);
L298NMotorDriver driver_M2(M2_IN1, M2_IN2, M2_ENA, M2_PWM_CNL);
L298NMotorDriver driver_M3(M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL);

SimpleMotorController controller_M0(driver_M0, 1.);
SimpleMotorController controller_M1(driver_M1, 1.);
SimpleMotorController controller_M2(driver_M2, 1.);
SimpleMotorController controller_M3(driver_M3, 1.);

MotorControllerManager motor_controll_manager{&controller_M0}; // initializer list

MecanumKinematics4W kinematics(WHEELRADIUS, L_X, L_Y);

RobotController robot_controller(motor_controll_manager, kinematics);

RosHandler ros_handler(robot_controller);

void setup() {
    ros_handler.setup();
}

void loop() {
    delay(100);
    ros_handler.spin();
}
