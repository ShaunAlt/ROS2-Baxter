#!/bin/bash
# =============================================================================
# Baxter Workspace Re-Test File
# -
# Run this instead of calling colcon-build and source every time.
#
# Must be run after having already created the Baxter Bridge and having run
#  ros_ws/init.sh
# =============================================================================

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================

echo "Setting up for Retest with Baxter-Dev"

# Source ROS2
echo "| - Sourcing ROS2"
source /opt/ros/galactic/setup.bash | sed 's/^/\t| - /'

# Build Workspace
echo "| - Building Workspace"
colcon build --packages-select baxter_dev baxter_int_ros2 baxter_int_ros2_support | sed 's/^/\t| - /'

# Source Workspace
echo "| - Sourcing Workspace"
source install/setup.bash | sed 's/^/\t| - /'

echo "| - Complete"

# =============================================================================
# End of File
# =============================================================================
