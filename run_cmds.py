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
    cmd('baxter_init')
    cmd('baxter_bridge')
    return

# =============
# ROS1 - MoveIT
def ros1_moveit():
    ''' Creates the ROS1 MoveIT Nodes. '''
    tab()
    cmd('baxter_init')
    cmd('moveit_ws')
    cmd('roslaunch baxter_moveit_config move_group.launch')
    tab()
    time.sleep(10)
    cmd('baxter_init')
    cmd('roslaunch baxter_dev ros2_moveit.launch')
    return

# ================
# ROS1 - Tuck Arms
def ros1_tuck(tuck):
    ''' Tucks (`True`) or Untucks (`False`) Baxters Arms. '''
    cmd('baxter_init')
    cmd(f'rosrun baxter_tools tuck_arms.py -{ {True: "t", False: "u"}[tuck] }')
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
        help = 'Equivalent of "-u -b -c -g"'
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
