#include <Arduino.h>

#include <geometry_msgs/msg/twist.h>

#include "subscribers/cmd_vel_subscriber.hpp"
#include "kinematics/kinematics.hpp"

void cmd_vel_subscription_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    Serial.println("Linear X: ");
    Serial.println(msg->linear.x);
    Serial.println("Angular Z: ");
    Serial.println(msg->angular.z);
    BLA::Matrix<3> robotVelocity;
    robotVelocity(0) = msg->linear.x;
    robotVelocity(1) = msg->linear.y;
    robotVelocity(2) = msg->angular.z;
    BLA::Matrix<4> wheelVelocity = calculateWheelVelocity(robotVelocity);

    // Print wheel velocities
    for(int i=0; i<4; i++) {
        Serial.print("Wheel ");
        Serial.print(i);
        Serial.print(" velocity: ");
        Serial.println(wheelVelocity(i));
    }
}
