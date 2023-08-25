# Roboost Robotics Software Stack

🤖 Roboost - Affordable and Accessible Robotics Development 🚀

Welcome to the Roboost repository! This is the heart of a modular robotics software stack designed to make robotic development cost-effective and user-friendly. Originally named after my robot models (Roboost V1 and V2), I've expanded its scope. The entire software framework, detached from specific hardware, is now referred to as Roboost. 🌐

This repository contains the codebase for multiple components of the Roboost robotic system. Each Roboost component mimics a brain region with a similar function. For instance, the low-level motor control component aligns with the Primary Motor Cortex in the brain. Just as this brain region handles executing motion commands from higher-level functions, this component manages the intricacies of motion execution.

Goals of the Roboost project:

- Minimize the cost of robotic development with ROS2
- Provide an easy-to-use base for more complex systems
- Configurable for different types of robots

More information can be found on the dedicated blog posts:

- [Roboost V2 Robot Summary](https://jakobfriedl.tech/project-summary/)
- [Roboost V2 Showcase](https://technologiehub.at/project-posts/roboost-v2/)
- ...

Here is a little teaser of the Roboost V2 robot:

![Roboost V2](Roboost\docs\Roboost-Demo.gif)

## Key Features

The software accomplishes these goals through several key features:

- The core firmware is designed to run on an ESP32. ROS2 integration enables UDP communication over WiFi, eliminating the need for an expensive onboard processor.
- The software architecture is modular, making it effortless to swap and configure components based on the robot's hardware.
- Firmware projects are built using PlatformIO, an accessible and user-friendly framework.
- Seamless integration with ROS2 allows the utilization of existing packages for complex systems.
- The code is written with readability, maintainability, and scalability in mind.
- Comprehensive Doxygen documentation is available for all projects (and soon, hardware files too).

## Primary Motor Cortex Module

🧠 Primary Motor Cortex: Low-Level Motion Control 🏃‍♂️

🤖 **Features:**

- Configurable for various hardware setups
- Communication options include UDP or UART, with or without a board computer
- Virtual base classes streamline the implementation of essential robot design aspects like kinematics and motor control.

The Primary Motor Cortex contains code related to the motor control of the robotic system. It is also based on a PlatformIO project for an ESP32 and listens to the /cmd_vel topic in the ROS network. The received messages then are converted into individual motor speeds which then are used to control the given motors.

More information can be found in the following blog post:

- [Roboost Primary Motor Cortex Summary](https://technologiehub.at/project-posts/roboost-primary-motor-cortex/)

### Biology of the Primary Motor Cortex - Explanation by ChatGPT

>"The primary motor cortex serves as a command center in the brain for controlling voluntary movements. Located in the frontal lobe, it functions like a programmer sending out signals that guide body actions. When making a voluntary movement, the primary motor cortex receives information from the frontal brain areas, develops a movement plan, and transmits this plan as signals down the spinal cord to muscles, functioning akin to dynamic programming for movement control."

## Entorhinal Cortex Module

🧠 Entorhinal Cortex: Navigation Sensor Data Processing 🛰️

🤖 **Features:**

- Designed as a PlatformIO project for seamless development on an ESP32.
- Specialized in reading data from navigation-centric sensors like LiDAR and IMU.
- Publishes processed sensor data onto corresponding ROS topics.
- Ensures reliable and consistent navigation data for higher-level decision-making.
- Configurable to adapt to various robot configurations and sensor setups.

The Entorhinal Cortex is a PlatformIO project that contains the firmware for an ESP32. It is responsible for reading navigation-specific sensor data (for example LiDAR and IMU sensors) and publishing it onto the corresponding ROS topics.

More information can be found in the following blog post:

- [Roboost Entorhinal Cortex Summary](https://technologiehub.at/project-posts/roboost-entorhinal-cortex/)

### Biology of the Entorhinal Cortex - Explanation by ChatGPT

>"The entorhinal cortex, located in the brain's medial temporal lobe, serves as a central hub for processing sensory information. It filters and integrates data from various senses before sending it to the hippocampus for memory storage. This region also aids in spatial memory and navigation, utilizing grid cells to create mental maps of environments, and its interaction with the hippocampus has been compared to computer data management, highlighting the brain's complexity and flexibility."

## TODO

- Add references for PCBs and used electrical components
- Share CAD files for the robot projects
