SHELL := /bin/bash
.PHONY: clean build submodules deps rviz rqt run-openmower run-sim

submodules:
	git submodule init
	git submodule update

deps: submodules
	sudo apt update
	rosdep update
	rosdep install --from-paths src --ignore-src --default-yes -r

build:
	catkin_make_isolated

clean:
	rm -Rf build* devel*

rviz:
	export ROS_MASTER_URI=http://10.0.0.134:11311 &&	source devel_isolated/setup.bash &&rviz

rqt:
	export ROS_MASTER_URI=http://10.0.0.134:11311 &&	source devel_isolated/setup.bash &&rqt

run:
	export ROS_MASTER_URI=http://10.0.0.134:11311 &&	source devel_isolated/setup.bash  &&	roslaunch om_slam slam.launch
