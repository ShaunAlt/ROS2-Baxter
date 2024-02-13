#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Command Line Command Generator
-
Creates and runs commands on the Command Line.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

import argparse
import os
import pyautogui as pg # type: ignore
import time


# =============================================================================
# Constant Definitions
# =============================================================================

pg.FAILSAFE = False
CMD_DELAY: float = 0.5


# =============================================================================
# Key Command Definitions
# =============================================================================

# ===========
# Run Command
def cmd(command: str) -> None:
    ''' Runs a Command. '''
    pg.write(command)
    pg.press('enter')
    time.sleep(CMD_DELAY)
    return None

# ==============
# Create New Tab
def tab(window: bool = False) -> None:
    ''' Creates a New Tab. '''
    mod = {True: 'alt', False: 'shift'}[window]
    with pg.hold('ctrl'):
        with pg.hold(mod):
            pg.press('t')
    time.sleep(CMD_DELAY)
    return None


# =============================================================================
# Commands
# =============================================================================

# =============
# Baxter Bridge
def bridge():
    ''' Creates and runs the Baxter Bridge. '''
    print(
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + '|            Creating Baxter Bridge            |\n' \
        + '|----------------------------------------------|\n' \
        + '| 1. Creates a new terminal tab.               |\n' \
        + '| 2. Runs `baxter_init`.                       |\n' \
        + '| 3. Waits 2 seconds for the script to finish. |\n' \
        + '| 4. Runs `baxter_bridge`.                     |\n' \
        + '|----------------------------------------------|\n' \
        + '|          Press `ENTER` Key to Start          |\n' \
        + '|______________________________________________|\n'
    )
    input()
    tab()
    cmd('baxter_init')
    time.sleep(2)
    cmd('baxter_bridge')
    return

# ==================
# Create Baxter SRDF
def create_srdf():
    ''' Creates the Baxter SRDF. '''
    print(
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + '|                   Create the Baxter SRDF                   |\n' \
        + '|           NOTE: THIS ONLY NEEDS TO BE RUN ONCE.            |\n' \
        + '|------------------------------------------------------------|\n' \
        + '| 1. Creates a new terminal tab.                             |\n' \
        + '| 2. Runs `moveit_ws`.                                       |\n' \
        + '| 3. Goes to the `baxter_moveit_config` directory.           |\n' \
        + '| 4. Create a temporary file.                                |\n' \
        + '| 5. Uses `xacro` to create the SRDF as a temporary file.    |\n' \
        + '| 6. Saves the temporary file as `baxter.srdf`.              |\n' \
        + '| 7. Edits the modification rights of `baxter.srdf` to allow |\n' \
        + '|    read/write access.                                      |\n' \
        + '|------------------------------------------------------------|\n' \
        + '|                 Press `ENTER` Key to Start                 |\n' \
        + '|____________________________________________________________|\n'
    )
    input()
    sudo_pwd = input('Enter the Password for the SUDO User:')
    tab()
    cmd('moveit_ws')
    cmd('cd `rospack find baxter_moveit_config`')
    cmd('temp=$(mktemp)')
    cmd(
        'xacro --inorder `rospack find baxter_moveit_config`/config/baxter.' \
        + 'srdf.xacro left_electric_gripper:=true right_electric_gripper' \
        + ':=true left_tip_name:=left_gripper right_tip_name:=' \
        + 'right_gripper > $temp'
    )
    cmd('sudo cp $temp config/baxter.srdf')
    cmd(sudo_pwd)
    cmd('sudo chmod a=wr config/baxter.srdf')
    return

# ======================
# Create Image Streamers
def image():
    ''' Create Image Streamers. '''
    print(
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + '|                  Create Image Streamers                   |\n' \
        + '|-----------------------------------------------------------|\n' \
        + '| 1. Completes the following steps for each of the image    |\n' \
        + '|    streamer topics:                                       |\n' \
        + '|    - "right_hand_camera/image_data"                       |\n' \
        + '|        - The raw camera data from Baxters right hand      |\n' \
        + '|          camera.                                          |\n' \
        + '|    - "right_hand_camera/processed_table"                  |\n' \
        + '|        - Once the sweeper has identified the workspace    |\n' \
        + '|          table, this will show a warped image of just the |\n' \
        + '|          table and its contents.                          |\n' \
        + '|    - "right_hand_camera/processed_table/occupancy/bool"   |\n' \
        + '|        - Shows the boolean occupancy grid created by the  |\n' \
        + '|          sweeper.                                         |\n' \
        + '| 2. Creates a new terminal tab.                            |\n' \
        + '| 3. Runs `ros2_ws`.                                        |\n' \
        + '| 4. Creates the image streamer for that topic.             |\n' \
        + '|-----------------------------------------------------------|\n' \
        + '|                Press `ENTER` Key to Start                 |\n' \
        + '|___________________________________________________________|\n'
    )
    input()
    # go through all of the image streamers to create
    for topic in [
            'right_hand_camera/image_data',
            'right_hand_camera/processed_table',
            'right_hand_camera/processed_table/occupancy/bool',
    ]:
        tab()
        cmd('ros2_ws')
        cmd(f'ros2 run baxter_dev streamer {topic}')
    return

# =============
# ROS1 - MoveIT
def ros1_moveit():
    ''' Creates the ROS1 MoveIT Nodes. '''
    print(
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + '|                 Creating ROS1 MoveIT Nodes                 |\n' \
        + '|------------------------------------------------------------|\n' \
        + '|  1. Creates a new terminal tab.                            |\n' \
        + '|  2. Runs `baxter_init`.                                    |\n' \
        + '|  3. Waits 2 seconds for the script to finish.              |\n' \
        + '|  4. Runs `ros_ws`.                                         |\n' \
        + '|  5. Runs `moveit_ws`.                                      |\n' \
        + '|  6. Runs `cd_ros1`.                                        |\n' \
        + '|  7. Updates Environment Variables.                         |\n' \
        + '|     - CMAKE_PREFIX_PATH                                    |\n' \
        + '|     - LD_LIBRARY_PATH                                      |\n' \
        + '|     - PKG_CONFIG_PATH                                      |\n' \
        + '|     - PYTHONPATH                                           |\n' \
        + '|     - ROS_PACKAGE_PATH                                     |\n' \
        + '|     - ROSLISP_PACKAGE_DIRECTORIES                          |\n' \
        + '|  8. Sets the Robot Semantic Description (SRDF) File        |\n' \
        + '|     Parameter.                                             |\n' \
        + '|  9. Launches the `move_group` and `controller` moveit      |\n' \
        + '|     files.                                                 |\n' \
        + '| 10. Once the `move_group` file has launched, press `ENTER` |\n' \
        + '|     after green text appears stating "You can start        |\n' \
        + '|     planning now!".                                        |\n' \
        + '| 11. Once the line "MoveIT Controller Ready" is printed,    |\n' \
        + '|     you can start sending it data.                         |\n' \
        + '|------------------------------------------------------------|\n' \
        + '|                 Press `ENTER` Key to Start                 |\n' \
        + '|------------------------------------------------------------|\n' \
        + '| NOTE:                                                      |\n' \
        + '| - If any of the following errors occur, stop the launch    |\n' \
        + '|   script (CTRL+C) and re-run it (it will be the most       |\n' \
        + '|   recent command run in the active terminal):              |\n' \
        + '|   - "group \'left_arm\' is not found".                       |\n' \
        + '|   - "group \'right_arm\' is not found".                      |\n' \
        + '|   - "could not connect to "move_group" action server.      |\n' \
        + '| - If any of the following errors occur, stop the launch    |\n' \
        + '|   script (CTRL+C), and run the command `roslaunch          |\n' \
        + '|   baxter_moveit_config demo_baxter.launch`. Once the text  |\n' \
        + '|   "You can start planning now!" appears, quit that launch  |\n' \
        + '|   and re-run the previous command.                         |\n' \
        + '|   - "robot semantic description not found".                |\n' \
        + '| - Ignore any of the following errors:                      |\n' \
        + '|   - "action client not connected".                         |\n' \
        + '| - For all other errors, or continuously appearing errors   |\n' \
        + '|   that require restarts, restart the entire terminal and   |\n' \
        + '|   retry. If that doesn\'t work, restart the Baxter robot    |\n' \
        + '|   and/or the computer and the issue should be resolved.    |\n'
        + '|____________________________________________________________|\n'
    )
    input()
    tab()
    cmd('baxter_init')
    time.sleep(2)
    cmd('ros_ws')
    cmd('moveit_ws')
    cmd('cd_ros1')
    cmd('export CMAKE_PREFIX_PATH="${PWD}/devel:${CMAKE_PREFIX_PATH}"')
    cmd('export LD_LIBRARY_PATH="${PWD}/devel/lib:${LD_LIBRARY_PATH}"')
    cmd(
        'export PKG_CONFIG_PATH="${PWD}/devel/lib/pkgconfig:$' \
        + '{PKG_CONFIG_PATH}"'
    )
    cmd(
        'export PYTHONPATH="${PWD}/devel/lib/python3/dist-packages:$' \
        + '{PYTHONPATH}"'
    )
    cmd('export ROS_PACKAGE_PATH="${PWD}/src:${ROS_PACKAGE_PATH}"')
    cmd(
        'export ROSLISP_PACKAGE_DIRECTORIES="${PWD}/devel/share/common-lisp:' \
        + '${ROSLISP_PACKAGE_DIRECTORIES}"'
    )
    cmd(
        'rosparam set /robot_description_semantic -t `rospack find ' \
        + 'baxter_moveit_config`/config/baxter.srdf'
    )
    cmd('roslaunch baxter_dev ros2_moveit.launch')
    return

# ================
# ROS1 - Tuck Arms
def ros1_tuck(tuck):
    ''' Tucks (`True`) or Untucks (`False`) Baxters Arms. '''
    a, b, c = {True: ('t', 'Tuck', 1), False: ('u', 'Untuck', 0)}[tuck]
    print(
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + f'|                 {" "*c} Baxter Arms -  {b} {" "*c}            ' \
        + '      |\n' \
        + '|----------------------------------------------------------|\n' \
        + '| 1. Creates a new terminal tab.                           |\n' \
        + '| 2. Runs `baxter_init`.                                   |\n' \
        + '| 3. Waits 2 seconds for the script to finish.             |\n' \
        + f'| 4. Runs the `tuck_arms.py` script with the -{a} parameter  |\n' \
        + f'|    to {b.lower()} Baxter\'s Arms. {" "*c*2}                  ' \
        + '           |\n' \
        + '|----------------------------------------------------------|\n' \
        + '|                Press `ENTER` Key to Start                |\n' \
        + '|__________________________________________________________|\n' 
    )
    input()
    tab()
    cmd('baxter_init')
    time.sleep(2)
    cmd(f'rosrun baxter_tools tuck_arms.py -{a}')
    return

# ===========
# Run Sweeper
def sweeper():
    ''' Runs the ROS2 Sweeper. '''
    print(
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + '|          Run Sweeper           |\n' \
        + '|--------------------------------|\n' \
        + '| 1. Creates a new terminal tab. |\n' \
        + '| 2. Runs `ros2_ws`.             |\n' \
        + '| 3. Runs the sweeper script.    |\n' \
        + '|--------------------------------|\n' \
        + '|   Press `ENTER` Key to Start   |\n' \
        + '|________________________________|\n'
    )
    input()
    tab()
    cmd('ros2_ws')
    cmd('ros2 run baxter_dev sweeper')


# =============================================================================
# Mainloop
# =============================================================================
def main() -> None:
    # define arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-a',
        '--all',
        action = 'store_true',
        help = 'Equivalent of "-u -b -m"'
    )
    parser.add_argument(
        '-b', 
        '--bridge', 
        action = 'store_true',
        help = 'Creates the Baxter Bridge.'
    )
    parser.add_argument(
        '-i',
        '--image',
        action = 'store_true',
        help = 'Create Image Streamers for Viewing Camera Data.'
    )
    parser.add_argument(
        '-m', 
        '--moveit', 
        action = 'store_true',
        help = 'Creates the ROS1 Baxter MoveIT Nodes.'
    )
    parser.add_argument(
        '-s',
        '--srdf',
        action = 'store_true',
        help = 'Creates the Baxter.SRDF File. RUN ONLY ONCE.'
    )
    parser.add_argument(
        '-t', 
        '--tuck', 
        action = 'store_true',
        help = 'Tucks the arms of the Baxter robot.'
    )
    parser.add_argument(
        '-u', 
        '--untuck', 
        action = 'store_true',
        help = 'Untucks the arms of the Baxter robot.'
    )
    parser.add_argument(
        '-w',
        '--sweeper',
        action = 'store_true',
        help = 'Run the Baxter Sweeper Script.'
    )
    args = parser.parse_args()

    if args.srdf: create_srdf()
    if args.tuck: ros1_tuck(True)
    if args.untuck or args.all: ros1_tuck(False)
    if args.bridge or args.all: bridge()
    if args.moveit or args.all: ros1_moveit()
    if args.sweeper or args.all: sweeper()
    if args.image or args.all: image()

if __name__ == '__main__':
    main()
    

# =============================================================================
# End of File
# =============================================================================
