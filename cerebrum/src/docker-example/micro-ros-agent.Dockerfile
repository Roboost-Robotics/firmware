# Use the ROS 2 base image
FROM ros:humble

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update && rosdep update

# Source the ROS 2 installation
RUN . /opt/ros/${ROS_DISTRO}/setup.sh

# Create a workspace and download the micro-ROS tools
RUN mkdir microros_ws
WORKDIR /microros_ws
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
RUN rosdep install --from-path src --ignore-src -y

# Build micro-ROS tools and source them
# Download and build micro-ROS-Agent packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh

# Set the entrypoint to run the micro-ROS agent
# For Serial: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
# For UDP: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
ENTRYPOINT ["/bin/bash", "-c", ". /opt/ros/${ROS_DISTRO}/setup.sh && source install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"]

EXPOSE 8888