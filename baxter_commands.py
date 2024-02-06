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
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + '|       Creating Baxter Bridge       |\n' \
        + '|------------------------------------|\n' \
        + '| 1. Creates a new terminal tab.     |\n' \
        + '| 2. Runs `baxter_init`.             |\n' \
        + '| 3. Waits 2 seconds for the script  |\n' \
        + '|    to finish.                      |\n' \
        + '| 4. Runs `baxter_bridge`.           |\n' \
        + '|------------------------------------|\n' \
        + '|     Press `ENTER` Key to Start     |\n' \
        + '|____________________________________|\n'
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
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + '|        Create the Baxter SRDF         |\n' \
        + '| NOTE: THIS ONLY NEEDS TO BE RUN ONCE. |\n' \
        + '|---------------------------------------|\n' \
        + '| 1. Creates a new terminal tab.        |\n' \
        + '| 1. Runs `moveit_ws`.                  |\n' \
        + '| 2. Goes to the `baxter_moveit_config` |\n' \
        + '|    directory.                         |\n' \
        + '| 3. Create a temporary file.           |\n' \
        + '| 4. Uses `xacro` to create the SRDF as |\n' \
        + '|    a temporary file.                  |\n' \
        + '| 5. Saves the temporary file as        |\n' \
        + '|    `baxter.srdf`.                     |\n' \
        + '| 6. Edits the modification rights of   |\n' \
        + '|    `baxter.srdf` to allow read/write  |\n' \
        + '|    access.                            |\n' \
        + '|---------------------------------------|\n' \
        + '|      Press `ENTER` Key to Start       |\n' \
        + '|_______________________________________|\n'
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

# =============
# ROS1 - MoveIT
def ros1_moveit():
    ''' Creates the ROS1 MoveIT Nodes. '''
    print(
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + '|     Creating ROS1 MoveIT Nodes      |\n' \
        + '|-------------------------------------|\n' \
        + '|  1. Creates a new terminal tab.     |\n' \
        + '|  2. Runs `baxter_init`.             |\n' \
        + '|  3. Waits 2 seconds for the script  |\n' \
        + '|     to finish.                      |\n' \
        + '|  4. Runs `ros_ws`.                  |\n' \
        + '|  5. Runs `moveit_ws`.               |\n' \
        + '|  6. Runs `cd_ros1`.                 |\n' \
        + '|  7. Updates Environment Variables.  |\n' \
        + '|     - CMAKE_PREFIX_PATH             |\n' \
        + '|     - LD_LIBRARY_PATH               |\n' \
        + '|     - PKG_CONFIG_PATH               |\n' \
        + '|     - PYTHONPATH                    |\n' \
        + '|     - ROS_PACKAGE_PATH              |\n' \
        + '|     - ROSLISP_PACKAGE_DIRECTORIES   |\n' \
        + '|  8. Sets the Robot Semantic         |\n' \
        + '|     Description (SRDF) File         |\n' \
        + '|     Parameter.                      |\n' \
        + '|  9. Launches the `move_group` and   |\n' \
        + '|     `controller` moveit files.      |\n' \
        + '| 10. Once the `move_group` file has  |\n' \
        + '|     launched, press `ENTER` after   |\n' \
        + '|     green text appears stating      |\n' \
        + '|     "You can start planning now!".  |\n' \
        + '| 11. Once the line "MoveIT           |\n' \
        + '|     Controller Ready" is printed,   |\n' \
        + '|     you can start sending it data.  |\n' \
        + '|-------------------------------------|\n' \
        + '|     Press `ENTER` Key to Start      |\n' \
        + '|_____________________________________|\n' \
        + '\n' \
        + 'NOTE:\n' \
        + '| - If any of the following errors occur, stop the launch\n' \
            + '|\tscript (CTRL+C) and re-run it (it will be the most\n' \
            + '|\trecent command run in the active terminal):\n' \
            + '|\t| - "group \'left_arm\' is not found".\n' \
            + '|\t| - "group \'right_arm\' is not found".\n' \
            + '|\t| - "could not connect to "move_group" action server.\n' \
        + '| - If any of the following errors occur, stop the launch\n' \
            + '|\tscript (CTRL+C), and run the command `roslaunch\n' \
            + '|\tbaxter_moveit_config demo_baxter.launch`. Once the text\n' \
            + '|\t"You can start planning now!" appears, quit that launch\n' \
            + '|\tand re-run the previous command.\n' \
            + '|\t| - "robot semantic description not found".\n' \
        + '| - Ignore any of the following errors:\n' \
            + '|\t| - "action client not connected".\n' \
        + '| - For all other errors, or continuously appearing errors that\n' \
            + '|\trequire restarts, restart the entire terminal and retry.\n' \
            + '|\tIf that doesn\'t work, restart the Baxter robot and/or\n' \
            + '|\tthe computer and the issue should be resolved.\n'
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
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + f'|      {" "*c} Baxter Arms -  {b} {" "*c}       |\n' \
        + '|------------------------------------|\n' \
        + '| 1. Creates a new terminal tab.     |\n' \
        + '| 2. Runs `baxter_init`.             |\n' \
        + '| 3. Waits 2 seconds for the script  |\n' \
        + '|    to finish.                      |\n' \
        + '| 4. Runs the `tuck_arms.py` script  |\n' \
        + f'|    with the -{a} parameter to        |\n' \
        + f'|    {b.lower()} Baxter\'s Arms.           {" "*c*2}|\n' \
        + '|------------------------------------|\n' \
        + '|     Press `ENTER` Key to Start     |\n' \
        + '|____________________________________|\n' 
    )
    input()
    tab()
    cmd('baxter_init')
    time.sleep(2)
    cmd(f'rosrun baxter_tools tuck_arms.py -{a}')
    return


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
    args = parser.parse_args()

    if args.srdf: create_srdf()
    if args.tuck: ros1_tuck(True)
    if args.untuck or args.all: ros1_tuck(False)
    if args.bridge or args.all: bridge()
    if args.moveit or args.all: ros1_moveit()

if __name__ == '__main__':
    main()
    

# =============================================================================
# End of File
# =============================================================================
