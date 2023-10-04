# Primary Motor Cortex Documentation

This repository is part of the Roboost project. For more information visit the [project website](https://www.technologiehub.at/Roboost) and the [Roboost GitHub organization](https://github.com/Roboost-Robotics).

## Description

This repository contains the code for the primary motor cortex of the robot. It is a PlatformIO project responsible for the control of a robot's motors. The [micro-ROS](https://micro.ros.org/) framework is used to connect the robot to a [ROS2](https://docs.ros.org/en/foxy/Installation.html) network. The robot can then be controlled using any ROS2 node publishing to /cmd_vel, such as the teleop_twist_keyboard.

## Configuration

For pin and hardware configuration (things like wheel dimensions, track width, etc.), changes can be made in the [conf_hardware.h](conf/conf_hardware.h) file. To change the hardware drivers, the [core.cpp](src/core.cpp) file needs to be modified. The [conf_network_example.h](conf/conf_network_example.h) file can be used to configure the network settings. Rename the file to `conf_network.h` and fill in the apropriate values to use it.

As mentioned in the [Installation](#installation) section, the micro-ROS agent can be configured to use either a wifi or serial connection. The default configuration is to use a wifi connection. To use a serial connection, the [platformio.ini](platformio.ini) file needs to be modified. Remove `board_microros_transport = wifi` and adapt the [core.cpp](src/core.cpp) file to use the serial connection.

### Supported Hardware

The primary motor cortex is designed to be modular. This means that the code can be easily adapted to different hardware configurations. The following components can be configured:

#### Motor Drivers

- L298N
- [VESCs](https://vesc-project.com/) (Currently in development)

#### Motor Controllers

- Simple Controller
  - Directly controlling the motors
- PID Controller
  - Using half quad encoders as feedback
  - Different tuning methods (Currently in development)

#### Kinematics

- 4-Wheeled Meccanum Drive
- 3-Wheeled swerve drive (Currently in development)

## Installation

The project can be installed as a standalone project or as part of the [Roboost-Cerebrum](TODO) repository. In case of the latter, the micro-ROS agent will be executed automatically as a Docker container. If you only want to use the primary motor cortex, you can install it as a standalone project. In this case, you will need to install the micro-ROS agent manually.

### Standalone Installation

#### Prerequisites

- [VSCode](https://code.visualstudio.com/)
- [PlatformIO](https://platformio.org/)
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [micro-ROS Agent](https://micro.ros.org/docs/tutorials/core/first_application_linux/)

#### Building

To install the project, clone the repository and open it in VSCode. The project is written for the [ESP32](https://www.espressif.com/en/products/socs/esp32) microcontroller, so you will need to install the [PlatformIO](https://platformio.org/) extension for VSCode. Once installed, you can build and upload the code to the microcontroller using the PlatformIO extension. All dependencies will be installed automatically.

#### Usage

To use the project, you will need to build the micro-ROS agent and run it on your host machine. The agent will communicate with the microcontroller via a wifi or serial connection. The agent will then publish and subscribe to ROS2 topics, which can be used to control the robot.

##### Micro-ROS Agent Installation

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

#### Running the Robot

Once the micro-ROS agent is running, you can run the robot using the following command:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

This will allow you to control the robot using the keyboard. You can also use any other ROS2 node to control the robot. Per default, the robot will subscribe to the `/cmd_vel` topic and publish to the `/odom` topic.

### Roboost-Cerebrum Installation

When installing the project as part of the [Roboost-Cerebrum](TODO) repository, the micro-ROS agent will be executed automatically as a Docker container. This means that you will not need to install the agent manually. In this case, you only need to adapt the project according to your hardware configuration, upload the code to the microcontroller and run the Roboost-Cerebrum Docker container. For more information, visit the [Roboost-Cerebrum](TODO) repository.

## TODO

- Add documentation of the motor shield PCB
- Add URDF listener (if not available, use configuration file)
