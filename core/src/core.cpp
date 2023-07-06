#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rcl_checks.h"

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include "robot_controller.hpp"
#include "motor_control/simple_motor_controller.hpp"
#include "motor_control/motor_drivers/l298n_motor_driver.hpp"
#include "conf_robot.h"
#include "conf_network.h"

L298NMotorDriver driver_M0(M0_IN1, M0_IN2, M0_ENA, M0_PWM_CNL);
L298NMotorDriver driver_M1(M1_IN1, M1_IN2, M1_ENA, M1_PWM_CNL);
L298NMotorDriver driver_M2(M2_IN1, M2_IN2, M2_ENA, M2_PWM_CNL);
L298NMotorDriver driver_M3(M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL);

SimpleMotorController controller_M0(driver_M0, 1.);
SimpleMotorController controller_M1(driver_M1, 1.);
SimpleMotorController controller_M2(driver_M2, 1.);
SimpleMotorController controller_M3(driver_M3, 1.);

MotorControllerManager motor_controll_manager{&controller_M0}; // initializer list

// TODO: initialize kinematics
MecanumKinematics4W kinematics(WHEELRADIUS, WHEEL_BASE, TRACK_WIDTH);
RobotController robot_controller(motor_controll_manager, kinematics);

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg;
nav_msgs__msg__Odometry odom;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Update the callback function definition
void cmd_vel_subscription_callback(const void * msgin) {
    const auto* msg = reinterpret_cast<const geometry_msgs__msg__Twist*>(msgin);

    // Convert the ROS Twist message to a BLA::Matrix<3> and call set_latest_command
    BLA::Matrix<3> cmd;
    cmd(0) = msg->linear.x;
    cmd(1) = msg->linear.y;
    cmd(2) = msg->angular.z;

    Serial.print("Linear X: ");
    Serial.println(cmd(0));
    Serial.print("Linear >: ");
    Serial.println(cmd(1));
    Serial.print("Angular Z: ");
    Serial.println(cmd(2));

    robot_controller.set_latest_command(cmd);
}


void setup() {
    // Configure serial transport
    Serial.begin(115200);
    IPAddress agent_ip(AGENT_IP);
    size_t agent_port = AGENT_PORT;

    set_microros_wifi_transports(SSID, SSID_PW, agent_ip, agent_port);
    delay(2000);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "roboost_core_node", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_subscription_callback, ON_NEW_DATA));

      // Create publisher for odometry
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"
    ));
}

void loop() {
    delay(100);
    // Publish the RobotController's latest odometry
    BLA::Matrix<6> odometry = robot_controller.get_odometry();

    // Convert the odometry matrix to a nav_msgs__msg__Odometry
    odom.pose.pose.position.x = odometry(0); // x position
    odom.pose.pose.position.y = odometry(1); // y position
    odom.pose.pose.orientation.z = odometry(2); // yaw orientation (using z-axis rotation)

    odom.twist.twist.linear.x = odometry(3); // x linear velocity
    odom.twist.twist.linear.y = odometry(4); // y linear velocity
    odom.twist.twist.angular.z = odometry(5); // z angular velocity

    RCSOFTCHECK(rcl_publish(&publisher, &odom, NULL));
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}