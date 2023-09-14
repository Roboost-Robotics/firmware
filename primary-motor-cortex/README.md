# Primary Motor Cortex Documentation

## Description

This repository contains the code for the primary motor cortex of the robot. It is responsible for the control of the robot's motors. It is a PlatformIO project written using the [micro-ROS](https://micro.ros.org/) framework.

## Installation

### Prerequisites

- [VSCode](https://code.visualstudio.com/)
- [PlatformIO](https://platformio.org/)
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [micro-ROS Agent](https://micro.ros.org/docs/tutorials/core/first_application_linux/)

### Building

To install the project, clone the repository and open it in VSCode. The project is written for the [ESP32](https://www.espressif.com/en/products/socs/esp32) microcontroller, so you will need to install the [PlatformIO](https://platformio.org/) extension for VSCode. Once installed, you can build and upload the code to the microcontroller using the PlatformIO extension. All dependencies will be installed automatically.

## Usage

To use the project, you will need to build the micro-ROS agent and run it on your host machine. The agent will communicate with the microcontroller via a wifi or serial connection. The agent will then publish and subscribe to ROS2 topics, which can be used to control the robot.

### Micro-ROS Agent Installation

```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash 
```

Once the micro-ROS agent is built, run following command to make the robot accessible over the network:

```bash
cd microros_ws
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
```

Alternatively, you can use the following command to use a serial connection:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

Note that depending on the communication method, you will need to modify the firmware of the microcontroller. Per default, the firmware is configured to use UDP over wifi.

### Running the Robot

Once the micro-ROS agent is running, you can run the robot using the following command:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

This will allow you to control the robot using the keyboard. You can also use any other ROS2 node to control the robot. Per default, the robot will subscribe to the `/cmd_vel` topic and publish to the `/odom` topic.
