#!/bin/bash
set -e

# Setup ROS 2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Execute command passed to the entrypoint
exec "$@"
