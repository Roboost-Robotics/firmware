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

#include <ros2arduino.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include "config.h"

#define SSID       ""
#define SSID_PW    ""
#define AGENT_IP   "" // IP of the ROS2 messages recceiver
#define AGENT_PORT 2018 //AGENT port number

#define PUBLISH_FREQUENCY 2 //hz

void publishString(std_msgs::String* msg, void* arg)
{
  (void)(arg);

  static int cnt = 0;
  sprintf(msg->data, "Hello ros2arduino %d", cnt++);
}

class StringPub : public ros2::Node
{
public:
  StringPub()
  : Node("ros2arduino_pub_node")
  {
    ros2::Publisher<std_msgs::String>* publisher_ = this->createPublisher<std_msgs::String>("arduino_chatter");
    this->createWallFreq(PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishString, nullptr, publisher_);
  }
};

WiFiClient client;

void setup() 
{
  WiFi.begin(SSID, SSID_PW);
  while(WiFi.status() != WL_CONNECTED);

  ros2::init(&client, AGENT_IP, AGENT_PORT);
}

void loop() 
{
  static StringPub StringNode;

  ros2::spin(&StringNode);
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