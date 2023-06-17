#include "subscribers/cmd_vel_subscriber.hpp"
#include "kinematics/kinematics.hpp"

// Subscriber callback function implementation
void cmd_vel_subscriber_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    Serial.print("Received linear velocity: ");
    Serial.print("Linear X: ");
    Serial.print(msg->linear.x);
    Serial.print(", Linear Y: ");
    Serial.println(msg->linear.y);
    Serial.print("Received angular velocity: ");
    Serial.println(msg->angular.z);

    // Calculate wheel velocities
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
