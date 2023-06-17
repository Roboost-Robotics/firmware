# Micro-ROS Examples

## Configuration
Rename the `conf_network_example.h` to `conf_network.h` and enter your network information.

## Upload Firmware

Connect your ESP32 board and click upload. Then press the EN button to start the micro-ROS node.

## Micro-ROS Agent

### Installation

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

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash 
```
### Execution

Use following commands to start the micro-ROS agent on your host PC to recceive the ROS messages. Change the values for `--dev` and `--port` according to your code configuration.

#### For Serial
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

#### For UDP
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```