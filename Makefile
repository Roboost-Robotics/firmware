all: help

help:
	@echo ""
	@echo "   Help Menu"
	@echo ""
	@echo "   setup_microros: Setup micro-ROS Agent"
	@echo "   ros_setup: Source ROS setup script"
	@echo "   clone_microros: Clone micro-ROS setup repository"
	@echo "   build_microros_agent: Build micro-ROS Agent"
	@echo "   pull: Pull changes from remote repository"
	@echo "   status: Show status of the repository"
	@echo "   help: Show this help menu"
	@echo ""

pull:
	git pull origin
	git --work-tree=./micro_ros_setup pull origin

status:
	git status
	git --work-tree=./micro_ros_setup status

setup_microros: ros_setup clone_microros build_microros_agent

ros_setup:
	@echo "Sourcing ROS setup script..."
	@. /opt/ros/${ROS_DISTRO}/setup.sh

clone_microros:
	@echo "Setting up workspace..."
	rm -rf micro_ros_setup && \
	git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git micro_ros_setup
	sudo apt update && rosdep update && \
	rosdep install --from-paths src --ignore-src -y

build_microros_agent:
	@echo "Building micro-ROS Agent..."
	cd ${ROBOOST_ROOT_DIR} && \
	colcon build --packages-select micro_ros_agent && \
	. install/local_setup.sh && \
	ros2 run micro_ros_setup create_agent_ws.sh && \
	ros2 run micro_ros_setup build_agent.sh && \
	. install/local_setup.sh

clone:
	git clone git@github.com:Roboost-Robotics/roboost_utils.git lib/utils
	git clone git@github.com:Roboost-Robotics/roboost_motor_control.git lib/motor_control
	git clone git@github.com:Roboost-Robotics/roboost_kinematics.git lib/kinematics

clean:
	rm -rf .pio