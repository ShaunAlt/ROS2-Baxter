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
import pyautogui as pg # type: ignore
import time


# =============================================================================
# Constant Definitions
# =============================================================================

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

# ============
# Echo Command
def echo(s: str) -> None:
    ''' Echos a string to the console. '''
    s = s.replace('"', '\\"')
    pg.write(f'echo "\n{s}"')
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
    tab()
    echo(
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + '|       Creating Baxter Bridge       |\n' \
        + '|------------------------------------|\n' \
        + '| 1. Runs `baxter_init`.             |\n' \
        + '| 2. Waits 5 seconds for the script  |\n' \
        + '|    to finish.                      |\n' \
        + '| 3. Runs `baxter_bridge`.           |\n' \
        + '|____________________________________|\n'
    )
    cmd('baxter_init')
    time.sleep(5)
    cmd('baxter_bridge')
    return

# =============
# ROS1 - MoveIT
def ros1_moveit():
    ''' Creates the ROS1 MoveIT Nodes. '''
    tab()
    echo(
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + '|     Creating ROS1 MoveIT Nodes     |\n' \
        + '|------------------------------------|\n' \
        + '| 1. Runs `baxter_init`.             |\n' \
        + '| 2. Waits 5 seconds for the script  |\n' \
        + '|    to finish.                      |\n' \
        + '| 3. Runs `moveit_ws`.               |\n' \
        + '| 4. Launches the MoveIT Move Group. |\n' \
        + '| 5. Waits 60 seconds for the        |\n' \
        + '|    `move_group.launch` file to     |\n' \
        + '|    launch.                         |\n' \
        + '| 6. Creates a new terminal tab.     |\n' \
        + '| 7. Runs `baxter_init`.             |\n' \
        + '| 8. Waits 5 seconds for the script  |\n' \
        + '|    to finish.                      |\n' \
        + '| 9. Launches the MoveIT Controller. |\n' \
        + '|____________________________________|\n' \
        + '\n' \
        + 'NOTE:\n' \
        + '| - If the error "robot semantic description not found" occurs,\n' \
            + '\tstop the script (CTRL+C) and run\n' \
            + '\t`roslaunch baxter_moveit_config demo_baxter.launch` to\n' \
            + '\topen RVIZ, which should fix the issue.\n' \
        + '| - Ignore "action client not connected" errors\n' \
        + '| - If the error "group \'{left_arm|right_arm}\' is not found\n' \
            + '\toccurs, restart both the `move_group` and `ros2_moveit`\n' \
            + '\tlaunch files.\n' \
        + '| - For all other errors, or continuing errors, restart machine\n' \
            + '\tand/or Baxter.\n'
    )
    cmd('baxter_init')
    time.sleep(5)
    cmd('moveit_ws')
    cmd('roslaunch baxter_moveit_config move_group.launch')
    time.sleep(60)
    tab()
    cmd('baxter_init')
    time.sleep(5)
    cmd('roslaunch baxter_dev ros2_moveit.launch')
    return

# ================
# ROS1 - Tuck Arms
def ros1_tuck(tuck):
    ''' Tucks (`True`) or Untucks (`False`) Baxters Arms. '''
    a, b, c = {True: ('t', 'Tuck', 1), False: ('u', 'Untuck', 0)}[tuck]
    tab()
    echo(
        '.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.\n' \
        + f'|      {" "*c} Baxter Arms -  {b} {" "*c}       |\n' \
        + '|------------------------------------|\n' \
        + '| 1. Runs `baxter_init`.             |\n' \
        + '| 2. Waits 5 seconds for the script  |\n' \
        + '|    to finish.                      |\n' \
        + '| 3. Runs the `tuck_arms.py` script  |\n' \
        + f'|    with the -{a} parameter to        |\n' \
        + f'|    {b.lower()} Baxter\'s Arms.           {" "*c*2}|\n' \
        + '|____________________________________|\n' 
    )
    cmd('baxter_init')
    time.sleep(5)
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
        '-u', 
        '--untuck', 
        action = 'store_true',
        help = 'Untucks the arms of the Baxter robot.'
    )
    parser.add_argument(
        '-t', 
        '--tuck', 
        action = 'store_true',
        help = 'Tucks the arms of the Baxter robot.'
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
    args = parser.parse_args()

    if args.tuck: ros1_tuck(True)
    if args.untuck or args.all: ros1_tuck(False)
    if args.bridge or args.all: bridge()
    if args.moveit or args.all: ros1_moveit()

if __name__ == '__main__':
    main()
    

# =============================================================================
# End of File
# =============================================================================
