all: help

help:
	@echo ""
	@echo "   Help Menu"
	@echo ""
	@echo "   make clone            - clones the subrepositories needed"
	@echo "   make pull             - pull the subrepositories"
	@echo "   make setup            - set up the micro-ROS environment and build the agent"
	@echo ""

pull:
	git pull origin
	git --work-tree=./micro_ros_setup pull origin

status:
	git status
	git --work-tree=./micro_ros_setup status

setup: ros_setup clone build_agent

ros_setup:
	@echo "Sourcing ROS setup script..."
	@. /opt/ros/${ROS_DISTRO}/setup.sh

clone:
	@echo "Setting up workspace..."
	rm -rf micro_ros_setup && \
	git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git micro_ros_setup
	sudo apt update && rosdep update && \
	rosdep install --from-paths src --ignore-src -y

build_agent:
	@echo "Building micro-ROS Agent..."
	cd ${ROBOOST_ROOT_DIR} && \
	colcon build --packages-select micro_ros_agent && \
	. install/local_setup.sh && \
	ros2 run micro_ros_setup create_agent_ws.sh && \
	ros2 run micro_ros_setup build_agent.sh && \
	. install/local_setup.sh
