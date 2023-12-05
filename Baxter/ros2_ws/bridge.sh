#!/bin/bash
# =============================================================================
# Baxter Initialization File
# -
# Run this instead of having to type out multiple commands at the start of each
#  session working with Baxter.
#
# MUST BE RUN AFTER ALREADY RUNNING ros_ws/init.sh
# =============================================================================

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================

# Source Workspace
source install/setup.bash

# Run Baxter Bridge
ros2 run baxter_bridge bridge -s

# =============================================================================
# End of File
# =============================================================================
