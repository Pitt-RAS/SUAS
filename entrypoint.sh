#!/bin/bash
# adapted from OSRF ros-melodic core image
set -e

# setup ros environment
source "/catkin_ws/devel/setup.bash"
exec "$@"