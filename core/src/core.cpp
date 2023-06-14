/**
 * @file core.cpp
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief 
 * @version 0.2
 * @date 2023-06-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

#ifdef ENCODERS
// Encoder specific definitions and functions

  // B pin of encoder is not used. The direction of the motors will be deduced from H-Bridge. This is, however, a precision flaw
  volatile uint16_t count_BL = 0;
  volatile uint16_t count_BR = 0;
  volatile uint16_t count_FL = 0;
  volatile uint16_t count_FR = 0;

  // Interrup routines
  void IRAM_ATTR function_ISR_EC_BL() {
    // Encoder out A triggers interrupt
    // TODO: check last B state to determine direction
    count_BL++;
  }

  void IRAM_ATTR function_ISR_EC_BR() {
    count_BR++;
  }

  void IRAM_ATTR function_ISR_EC_FL() {
    count_FL++;
  }

  void IRAM_ATTR function_ISR_EC_FR() {
    count_FR++;
  }

#else

#endif

#if defined(MECANUM_4WHEEL)
  /**
   * @brief Calculates wheel velocity based on given robot velocity
   * 
   * @param robotVelocity 
   * @return BLA::Matrix<4> 
   */
  BLA::Matrix<4> calculateWheelVelocity(BLA::Matrix<3> robotVelocity){
    
    BLA::Matrix<4> wheelVelocity;
    BLA::Matrix<4, 3> forwardKinematicsModel = { 1, -1, -(L_X + L_Y),
                                                1, 1, L_X + L_Y,
                                                1, 1, -(L_X + L_Y),
                                                1, -1, L_X + L_Y};
    wheelVelocity = forwardKinematicsModel * robotVelocity;
    wheelVelocity *=  1 / WHEELRADIUS;

    return wheelVelocity;
  }

  /**
   * @brief Calculates the velocity in direction x and y, as well as the angular velocity around the z axis [m/s] [m/s] [rad/s].
   * 
   * @param wheelVelocity 
   * @return BLA::Matrix<3> velocity of the robot in x, y and rotational velocity around z
   */
  BLA::Matrix<3> calculateRobotVelocity(BLA::Matrix<4> wheelVelocity){
    BLA::Matrix<3> robotVelocity;

    BLA::Matrix<3, 4> inverseKinematicsModel = { 1, 1, 1, 1, 
                                                -1, 1, 1, -1, 
                                                -1/(L_X + L_Y), 1/(L_X + L_Y), -1/(L_X + L_Y), 1/(L_X + L_Y)};

    robotVelocity = inverseKinematicsModel * wheelVelocity;
    robotVelocity *= WHEELRADIUS / 4;

    return robotVelocity;
  }
#elif defined(SWERVE_3WHEEL)
    // TODO
#else
    // TODO: throw compile error
#endif