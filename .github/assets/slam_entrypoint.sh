#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/om_slam/devel_isolated/setup.bash
exec "$@"
