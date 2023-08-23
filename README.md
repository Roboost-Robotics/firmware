# Roboost Documentation
![Roboost Logo](Roboost-logo.svg)

# Roboost Firmware Collection

This repository contains the codebase for multiple components of the Roboost robotic system.
The naming of each part of the system is based on regions of the brain that are responsible for similar tasks.

Goals of the Roboost project:
- Minimize the cost of robotic development with ROS2
- Provide an easy-to-use base for more complex systems
- Configurable for different types of robots

More information can be found on the dedicated blog posts:

- [Roboost V2 Robot Summary](https://jakobfriedl.tech/project-summary/)
- [Roboost V2 Showcase](https://technologiehub.at/project-posts/roboost-v2/)
- ...

## Entorhinal Cortex

The Entorhinal Cortex is a PlatformIO project that contains the firmware for an ESP32. It is responsible for reading navigation-specific sensor data (for example LiDAR and IMU sensors) and publishing it onto the corresponding ROS topics.

More information can be found in the following blog post:

- [Roboost Entorhinal Cortex Summary](https://technologiehub.at/project-posts/roboost-entorhinal-cortex/)

### Biology of the Entorhinal Cortex - Explanation by ChatGPT

>"The entorhinal cortex, located in the brain's medial temporal lobe, serves as a central hub for processing sensory information. It filters and integrates data from various senses before sending it to the hippocampus for memory storage. This region also aids in spatial memory and navigation, utilizing grid cells to create mental maps of environments, and its interaction with the hippocampus has been compared to computer data management, highlighting the brain's complexity and flexibility."

## Primary Motor Cortex

The Primary Motor Cortex contains code related to the motor control of the robotic system. It is also based on a PlatformIO project for an ESP32 and listens to the /cmd_vel topic in the ROS network. The received messages then are converted into individual motor speeds which then are used to control the given motors.

More information can be found in the following blog post:

- [Roboost Primary Motor Cortex Summary](https://technologiehub.at/project-posts/roboost-primary-motor-cortex/)

### Biology of the Primary Motor Cortex - Explanation by ChatGPT

>"The primary motor cortex serves as a command center in the brain for controlling voluntary movements. Located in the frontal lobe, it functions like a programmer sending out signals that guide body actions. When making a voluntary movement, the primary motor cortex receives information from the frontal brain areas, develops a movement plan, and transmits this plan as signals down the spinal cord to muscles, functioning akin to dynamic programming for movement control."

## Project Structure

|--entorhinal-cortex
|  |-README.md
|  |--conf
|  |  |-README.md
|  |  |-conf_hardware.h
|  |  |-conf_network_example.h
|  |  |-conf_network.h
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
|  |-platformio.ini
|--primary-motor-cortex
|  |-README.md
|  |--conf
|  |  |-README.md
|  |  |-conf_hardware.h
|  |  |-conf_network_example.h
|  |  |-conf_network.h
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
|  |-platformio.ini
|-README.md
|-Roboost-logo.svg

# Documentation

Welcome to the documentation folder of the Roboost Primary Motor Cortex.

This folder contains documentation related to the components's codebase. The documentation is generated using Doxygen, a powerful tool for automatically generating documentation from source code comments.

## Generating Documentation

To generate the documentation for this project, follow these steps:

1. **Install Doxygen:**
   If you haven't already, make sure you have Doxygen installed on your system. You can download and install it from the official Doxygen website (<https://www.doxygen.nl/download.html>) or using your system's package manager.

2. **Configure Doxygen:**
   The configuration for generating documentation is defined in the `Doxyfile` located in the project's root folder. You can adjust the settings in this file to tailor the documentation to your needs.

3. **Generate Documentation:**
   Open a terminal and navigate to the project's root folder.

    ```bash
    cd path/to/your/project#
    ```

    Run Doxygen with the Doxyfile as the input configuration file.

    ```bash
    doxygen Doxyfile
    ```

   Doxygen will process your source code comments and generate the documentation in the specified output directory (default: `./docs`).

4. **View Documentation:**
Once the documentation is generated, you can open the HTML files in a web browser to view the documentation. The main entry point is usually `index.html`.

## Documentation Output

The generated documentation is stored in the `doc` folder at the root of the project. You can find various HTML files, diagrams, and other resources related to the project's documentation in this folder.

Feel free to explore the generated documentation to understand the project's structure, classes, methods, and their relationships.

## Customizing Documentation

You can customize the documentation appearance, structure, and content by modifying the `Doxyfile` settings. Doxygen provides a wide range of options to tailor the documentation to your preferences.

For more information on Doxygen settings and customization, refer to the official Doxygen documentation (<https://www.doxygen.nl/manual/index.html>).

---

For more information about the project itself, please refer to the project's main README.md file.

If you encounter any issues or have questions regarding the documentation, feel free to open an issue on the project's GitHub repository.


## TODO

- Add references for PCBs and used electrical components
- Share CAD files for the robot projects