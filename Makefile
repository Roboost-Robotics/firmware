all: help

help:
	@echo ""
	@echo "   Help Menu"
	@echo ""
	@echo "   make clone            - clones the subrepostories needed"
	@echo "   make pull             - pull the subrepostories"
	@echo ""

pull:
	git pull origin
	git --work-tree=./micro_ros_setup pull origin

status:
	git status
	git --work-tree=./micro_ros_setup status
	
micro_ros_setup:
		git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git $@

clone:  \
	micro_ros_setup
	
