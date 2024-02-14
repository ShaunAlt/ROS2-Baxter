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

    update_upgrade_apt() {
        echo "Updating and Upgrading APT" | S1
        sudo apt update -y 2>&1 | S2
        sudo apt upgrade -y 2>&1 | S2
    }
    update_upgrade_apt
    echo "Add Ubuntu Universe Repository" | S1
    sudo apt-get install software-properties-common -y | S2
    sudo add-apt-repository universe -y | S2
    
    echo "Setup packages.ros.org" | S1
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' | S2

    echo "Install CURL" | S1
    sudo apt-get install curl -y | S2

    echo "Setup ROS Keys" | S1
    sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - | S2
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg | S2

    echo "Add ROS Key Repository" | S1
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null | S2

    update_upgrade_apt
    echo "Install ROS1 Noetic" | S1
    sudo apt-get install ros-noetic-desktop-full -y | S2

    echo "Install ROS1 Noetic Catkin" | S1
    sudo apt-get install ros-noetic-catkin -y | S2

    echo "Install ROS1 Noetic MoveIT" | S1
    sudo apt-get install ros-noetic-moveit -y | S2

    update_upgrade_apt
    echo "Install ROS2 Galactic" | S1
    sudo apt-get install ros-galactic-desktop -y | S2

    echo "Install ROS Developer Tools" | S1
    sudo apt-get install ros-dev-tools -y | S2

    echo "Install Python3 Catkin Tools" | S1
    sudo apt-get install python3-catkin-tools -y | S2

    update_upgrade_apt
    echo "Install Python3 PIP" | S1
    sudo apt-get install python3-pip -y | S2

    echo "Install Python3 ROSDEP" | S1
    sudo apt-get install python3-rosdep -y | S2

    echo "Install Python3 ROSINSTALL" | S1
    sudo apt-get install python3-rosinstall -y | S2

    echo "Install Python3 ROSINSTALL-GENERATOR" | S1
    sudo apt-get install python3-rosinstall-generator -y | S2

    echo "Install Python3 WSTOOL" | S1
    sudo apt-get install python3-wstool -y | S2

    echo "Install Build-Essential" | S1
    sudo apt-get install build-essential -y | S2

    update_upgrade_apt
    echo "Initialize ROSDEP" | S1
    sudo rosdep init | S2
    rosdep update | S2

    echo "Install PIP - PyAutoGUI" | S1
    pip install pyautogui==0.9.54 | S2

    echo "Install PIP - OpenCV" | S1
    pip install opencv-python==4.4.0.46 | S2
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
        "alias run_image='cd ${DIR} && ./baxter_commands.py -i'"
        "alias run_moveit='cd ${DIR} && ./baxter_commands.py -m'"
        "alias run_srdf='cd ${DIR} && ./baxter_commands.py -s'"
        "alias run_sweeper='cd ${DIR} && ./baxter_commands.py -w'"
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

# Go back to repository root
echo "| - Going back to Repository Root"
cd $DIR

# Explain next steps
echo "
Next Steps:
| - Source all of the Aliases created:
|   | - source ~/.bash_aliases
| - Create the Baxter SRDF by running the following commands:
|   | - run_srdf
"

# =============================================================================
# End of File
# =============================================================================
