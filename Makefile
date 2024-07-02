SHELL := /bin/bash
.PHONY: clean build submodules deps rviz rqt run-openmower run-sim abseil rosdeps

submodules:
	git submodule init
	git submodule update

rosdeps:
	sudo apt update
	rosdep update
	rosdep install --from-paths src --ignore-src --default-yes -r

deps: submodules rosdeps abseil

build:
	catkin_make_isolated --use-ninja

abseil:
	rm -Rf abseil-cpp && src/lib/cartographer/scripts/install_abseil.sh

clean:
	rm -Rf build_* devel_*

rviz:
	export ROS_MASTER_URI=http://10.0.0.134:11311 &&	source devel_isolated/setup.bash &&rviz

rqt:
	export ROS_MASTER_URI=http://10.0.0.134:11311 &&	source devel_isolated/setup.bash &&rqt

run:
	export ROS_MASTER_URI=http://10.0.0.134:11311 && source devel_isolated/setup.bash  &&	roslaunch om_slam slam.launch
