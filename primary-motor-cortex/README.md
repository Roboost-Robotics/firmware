/**
 * @subpage primary-motor-cortex
 *
 * This is the main documentation page for the Roboost project.
 *
 * @section intro Introduction
 * ...
 */
# Primary Motor Cortex Documentation

## System Design

- *MotorController*: This is an abstract base class that provides an interface for controlling a motor. It has a method for setting the control value for a motor (such as set_motor_control). The control value is a normalized value between -1 and 1.

- *SimpleMotorController*: This is a subclass of MotorController that provides a simple implementation. It uses a max velocity and maps this to the normalized control value.

- *MotorControllerManager*: This class contains a collection of MotorController objects, one for each wheel on the robot. It has methods for controlling all motors at once, which it delegates to each individual MotorController.

- *KinematicsModel*: This is an abstract base class that represents a kinematic model for the robot. It has a method to calculate the desired wheel speeds given a robot velocity and rotation speed.

- *RobotController*: This class uses a MotorControllerManager and a KinematicsModel to control the robot. It has an update() method that uses the KinematicsModel to calculate desired wheel speeds and the MotorControllerManager to set these speeds. It also has a get_odometry() method to provide odometry data for the robot.

- *ROSHandler*: This is a class that handles ROS communication. It uses a RobotController to control the robot based on incoming ROS messages and publishes the robot's odometry data. This class should be separated from the robot control logic for modularity and reusability.

In the main loop of your program, you would call the update() method of the RobotController to update the state of the robot based on the latest ROS messages, and use the get_odometry() method to provide odometry data to the ROS publisher.

This design allows you to easily switch out different components (like different MotorController or KinematicsModel implementations) and keeps the ROS-specific code separated from the rest of your robot code. It's modular, which promotes reusability, and it's flexible, allowing for various robot configurations and control strategies.

Data Storage:

todo GPIO pins will vary depending on the motor controller, so it does not make sense to store them at the base class.

- *MotorController*: This class would store the GPIO pin numbers associated with each motor control function (PWM, IN1, IN2). It would also store the current control value for the motor.

todo

- *SimpleMotorController*: In addition to the data stored by the MotorController base class, SimpleMotorController would store the max velocity for the motor.

- *MotorControllerManager*: This class would store a collection of MotorController objects. This could be an array, a vector, or some other container, depending on what makes sense for your setup.

- *KinematicsModel*: The data stored by this class would depend on the specifics of the kinematics model. For example, a differential drive kinematics model might store the wheelbase and wheel radius, while an omni-directional model might store different parameters.

- *RobotController*: This class would store an instance of MotorControllerManager and KinematicsModel. It could also store the current state of the robot (position, orientation, velocity, etc.), depending on how your system is set up.

- *ROSHandler*: This class would store the RobotController instance it's using to control the robot. It would also store any ROS-related data it needs, such as the topics it's subscribing to/publishing on, the current state of the ROS node, etc.

## Usage

nices tutorial:
<https://micro.ros.org/docs/tutorials/core/first_application_linux/>

to run:

- upload code
- press EN pin
- build micro-ROS agent:

```bash
# cd into git-cloned micro-ROS project folder
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

- once the micro-ROS agent is built, run following command to make the serial port accessible in the host machine:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

For wireless controll:

```bash
cd microros_ws
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
```

Then start teleop to controll the robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
