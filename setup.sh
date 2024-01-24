#!/bin/bash
# =============================================================================
# ROS2-Baxter Repository Setup File
# -
# Run this file once after first git cloning the repository to setup all of the
#  submodules as well as build all of the workspaces.
# =============================================================================

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================

# Initialize Variables
alias S1='sed "s/^/\t| - /"'
alias S2='sed "s/^/\t\t| - /"'
alias S3='sed "s/^/\t\t\t| - /"'
alias S4='sed "s/^/\t\t\t\t| - /"'
alias S5='sed "s/^/\t\t\t\t\t| - /"'
DIR=$(pwd)

# Print start of Commands
shopt -s expand_aliases
sudo echo "Setting up ROS2-Baxter Git Repo"

# Installing Dependencies
install_dependencies() {
    echo "| - Installing Dependencies"
    echo "Python3 Catkin Tools" | S1
    sudo apt-get install python3-catkin-tools | S2
    echo "ROS1 Noetic MoveIT" | S1
    sudo apt-get install ros-noetic-moveit | S2
    echo "ROS1 Noetic Catkin" | S1
    sudo apt-get install ros-noetic-catkin | S2
    echo "ROS2 Galactic Desktop" | S1
    sudo apt-get install ros-galactic-desktop | S2
    echo "ROS2 Galactic ROS Base" | S1
    sudo apt-get install ros-galactic-ros-base | S2
    echo "ROS Developer Tools" | S1
    sudo apt-get install ros-dev-tools | S2
}
install_dependencies

# Edit Aliases
create_aliases() {
    echo "| - Creating Aliases"
    echo "Defining Aliases" | S1
    ALIASES=(
        "alias baxter_bridge='ros2_ws && ./bridge.sh'"
        "alias baxter_connect='baxter_init && baxter_bridge'"
        "alias baxter_init='ros_ws && ./init.sh'"
        "alias baxter_ping='ping 011312P0012.local'"
        "alias cd_moveit='cd ${DIR}/Baxter/moveit_ws'"
        "alias cd_ros1='cd ${DIR}/Baxter/ros_ws'"
        "alias cd_ros2='cd ${DIR}/Baxter/ros2_ws'"
        "alias src_galactic='source /opt/ros/galactic/setup.bash'"
        "alias src_noetic='source /opt/ros/noetic/setup.bash'"
        "alias src_ws1='source devel/setup.bash'"
        "alias src_ws2='source install/setup.bash'"
        "alias moveit_ws='cd_moveit && src_ws1'"
        "alias ros_ws='cd_ros1 && src_ws1'"
        "alias ros2_ws='cd_ros2 && src_ws2'"
    )
    echo "Setting Aliases" | S1
    printf "%s\n" "${ALIASES[@]}" >> ~/.bash_aliases
    echo "Defining Functions" | S1
    FUNCTIONS=(
        "rosdep_udpate() { rosdep update --rosdistro \$1; }"
        "rosdep_install() { rosdep install -r --from-paths . --ignore-src --rosdistro \$1 -y; }"
    )
    echo "Setting Functions" | S1
    printf "%s\n" "${FUNCTIONS[@]}" >> ~/.bash_aliases
    echo "Sourcing Aliases" | S1
    source ~/.bash_aliases
}
create_aliases

# Clone the Submodules
echo "| - Cloning Submodules"
git submodule update --force --recursive --init --remote | S1

# Build Workspaces
build_workspaces() {
    build_ros1_workspaces() {
        echo "Sourcing ROS1 Noetic" | S2
        src_noetic | S3
        echo "Updating Noetic ROSDEP" | S2
        rosdep_update noetic | S3

        # MoveIT
        echo "Building MoveIT Workspace" | S2
        echo "Going to Workspace" | S3
        cd_moveit
        echo "Installing Noetic ROSDEPs" | S3
        cd src
        rosdep_install noetic | S4
        cd ..
        echo "Building Workspace" | S3
        catkin build | S4

        # ROS1
        echo "Building ROS1 Workspace" | S2
        echo "Going to Workspace" | S3
        cd_ros1
        echo "Installing Noetic ROSDEPs" | S3
        cd src
        rosdep_install noetic | S4
        cd ..
        echo "Building Workspace" | S3
        catkin_make | S4
    }
    build_ros2_workspaces() {
        echo "Sourcing ROS2 Galactic" | S2
        src_galactic | S3
        echo "Updating Galactic ROSDEP" | S2
        rosdep_update galactic | S3

        # ROS2
        echo "Building ROS2 Workspace" | S2
        echo "Going to Workspace" | S3
        cd_ros2
        echo "Cleaning up \`baxter_common_ros2\` Module" | S3
        cd src
        echo "Deleting Original" | S4
        rm -rf baxter_common_ros2
        echo "Git Cloning New" | S4
        git clone https://github.com/CentraleNantesRobotics/baxter_common_ros2.git | S5
        echo "Removing Redundant Baxter Bridge"
        rm -rf baxter_common_ros2/baxter_bridge
        echo "Installing Galactic ROSDEPs" | S3
        rosdep_install galactic | S4
        cd ..
        echo "Building Workspace" | S3
        colcon build | S4
    }
    echo "| - Building Workspaces"
    echo "Building Noetic Workspaces" | S1
    build_ros1_workspaces
    echo "Building Galactic Workspaces" | S1
    build_ros2_workspaces
}
build_workspaces

# Go back to repository root
echo "| - Going back to Repository Root"
cd $DIR

# =============================================================================
# End of File
# =============================================================================
