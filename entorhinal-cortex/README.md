/**
 * @subpage entorhinal-cortex
 *
 * This is the main documentation page for the Roboost project.
 *
 * @section intro Introduction
 * ...
 */

# Entorhinal Cortex Documentation

## System Design

TODO

## Usage

nices tutorial:
https://micro.ros.org/docs/tutorials/core/first_application_linux/

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

Structure:

|--entorhinal-cortex
|  |-README.md
|  |--conf
|  |  |-README.md
|  |  |-conf_hardware.h
|  |  |-conf_network_example.h
|  |  |-conf_network.h
|  |--doc
|  |  |-README.md
|  |--include
|  |  |-README.md
|  |  |-rcl_checks.h
|  |--lib
|  |  |-README.md
|  |--res
|  |  |-README.md
|  |  |-kinematics_calculations.mcdx
|  |--src
|  |  |-README.md
|  |  |-core.cpp
|  |-.clang-format
|  |-.gitignore
|  |-Doxyfile
|  |-platformio.ini
|--primary-motor-cortex
|  |-README.md
|  |--conf
|  |  |-README.md
|  |  |-conf_hardware.h
|  |  |-conf_network_example.h
|  |  |-conf_network.h
|  |--doc
|  |  |-README.md
|  |--include
|  |  |-README.md
|  |  |--kinematics
|  |  |  |-kinematics.hpp
|  |  |--motor-control
|  |  |  |--motor-drivers
|  |  |  |  |-motor_driver.hpp
|  |  |  |  |-l298n_motor_driver.hpp
|  |  |  |-encoder.hpp
|  |  |  |-motor_control_manager.hpp
|  |  |  |-motor_controller.hpp
|  |  |  |-pid.hpp
|  |  |  |-simple_motor_controller.hpp
|  |  |-rcl_checks.h
|  |  |-robot_controller.hpp
|  |--lib
|  |  |-README.md
|  |--res
|  |  |-README.md
|  |  |-kinematics_calculations.mcdx
|  |--src
|  |  |-README.md
|  |  |--kinematics
|  |  |  |-mecanum_kinematics_4w.cpp
|  |  |--motor-control
|  |  |  |--motor-drivers
|  |  |  |  |-l298n_motor_driver.cpp
|  |  |  |-encoder.cpp
|  |  |  |-motor_control_manager.cpp
|  |  |  |-pid.cpp
|  |  |  |-simple_motor_controller.cpp
|  |  |-core.cpp
|  |  |-robot_controller.cpp
|  |-.clang-format
|  |-.gitignore
|  |-Doxyfile
|  |-platformio.ini
|-README.md
|-Roboost-logo.svg