# README

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
```

Then start teleop to controll the robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```