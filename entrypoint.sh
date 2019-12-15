#!/bin/bash
# adapted from OSRF ros-melodic core image
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.sh"
exec "$@"