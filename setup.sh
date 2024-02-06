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
rosdep_update() { rosdep update --rosdistro $1; }
rosdep_install() { rosdep install -r --from-paths . --ignore-src --rosdistro $1 -y; }
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
    echo "Python3 PIP" | S1
    sudo apt-get install python3-pip | S2
    echo "PIP Packages" | S1
    echo "PyAutoGUI" | S2
    pip install pyautogui==0.9.54 | S3
    echo "OpenCV 2" | S2
    pip install opencv-python==4.4.0.46 | S3
}
install_dependencies

# Edit Aliases
create_aliases() {
    echo "| - Creating Aliases"
    echo "Defining Aliases" | S1
    ALIASES=(
        "alias baxter_bridge='ros2_ws && ./bridge.sh'"
        "alias baxter_init='ros_ws && ./init.sh'"
        "alias baxter_ping='ping 011312P0012.local'"
        "alias cd_moveit='cd ${DIR}/Baxter/moveit_ws'"
        "alias cd_ros1='cd ${DIR}/Baxter/ros_ws'"
        "alias cd_ros2='cd ${DIR}/Baxter/ros2_ws'"
        "alias src_galactic='. /opt/ros/galactic/setup.bash'"
        "alias src_noetic='. /opt/ros/noetic/setup.bash'"
        "alias src_ws1='. devel/setup.bash'"
        "alias src_ws2='. install/setup.bash'"
        "alias moveit_ws='cd_moveit && src_ws1'"
        "alias ros_ws='cd_ros1 && src_ws1'"
        "alias ros2_ws='cd_ros2 && src_ws2'"
        "alias run_all='cd ${DIR} && ./baxter_commands.py -a'"
        "alias run_bridge='cd ${DIR} && ./baxter_commands.py -b'"
        "alias run_moveit='cd ${DIR} && ./baxter_commands.py -m'"
        "alias run_tuck='cd ${DIR} && ./baxter_commands.py -t'"
        "alias run_untuck='cd ${DIR} && ./baxter_commands.py -u'"
    )
    echo "Setting Aliases" | S1
    printf "%s\n" "${ALIASES[@]}" >> ~/.bash_aliases
    echo "Sourcing Aliases" | S1
    source ~/.bash_aliases
    echo "All Aliases" | S1
    alias | S2
}
create_aliases

# Clone the Submodules
echo "| - Cloning Submodules"
sudo git submodule update --force --recursive --init --remote 2>&1 | S1

# Build Workspaces
build_workspaces() {
    build_ros1_workspaces() {
        echo "Sourcing ROS1 Noetic" | S2
        src_noetic > /dev/null 2>&1
        env | grep ROS_ | S3
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
        catkin build 2>&1 | S4

        # ROS1
        echo "Building ROS1 Workspace" | S2
        echo "Going to Workspace" | S3
        cd_ros1
        echo "Installing Noetic ROSDEPs" | S3
        cd src
        rosdep_install noetic | S4
        cd ..
        echo "Building Workspace" | S3
        catkin_make 2>&1 | S4
    }
    build_ros2_workspaces() {
        echo "Sourcing ROS2 Galactic" | S2
        src_galactic > /dev/null 2>&1
        env | grep ROS_ | S3
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
        git clone https://github.com/CentraleNantesRobotics/baxter_common_ros2.git 2>&1 | S5
        echo "Removing Redundant Baxter Bridge" | S4
        rm -rf baxter_common_ros2/baxter_bridge
        echo "Installing Galactic ROSDEPs" | S3
        rosdep_install galactic | S4
        cd ..
        echo "Building Workspace" | S3
        colcon build 2>&1 | S4
    }
    echo "| - Building Workspaces"
    echo "Building Noetic Workspaces" | S1
    build_ros1_workspaces
    echo "Building Galactic Workspaces" | S1
    build_ros2_workspaces
}
build_workspaces

echo "Setting up Baxter SRDF" | S1
moveit_ws | S2
cd src/moveit_robots/baxter/baxter_moveit_config
env | grep ROS_
sudo xacro --inorder `rospack find baxter_moveit_config`/config/baxter.srdf.xacro left_electric_gripper:=true right_electric_gripper:=true left_tip_name:=left_gripper right_tip_name:=right_gripper > config/baxter.srdf | S2

# Go back to repository root
echo "| - Going back to Repository Root"
cd $DIR

# =============================================================================
# End of File
# =============================================================================
