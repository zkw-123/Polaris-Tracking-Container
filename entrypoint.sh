#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace if the package isn't built yet
if [ ! -d "/ros2_ws/install/ros2_ndi_tracker" ]; then
  echo "Building ROS2 workspace...go go go"
  cd /ros2_ws
  colcon build --symlink-install
  echo "Workspace built successfully1!"
fi

# Source the workspace
source /ros2_ws/install/setup.bash

# Check if the executable exists
if ros2 pkg executables ros2_ndi_tracker | grep -q "ndi_tracker_node"; then
  # Execute the command passed to the entrypoint
  exec "$@"
else
  echo "Executable not found, running Python script directly"
  python3 /ros2_ws/src/ros2_ndi_tracker/ros2_ndi_tracker/ndi_tracker_node.py
fi