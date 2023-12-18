#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter Gripper Tester
-
Contains functions which are able to test various parts of the Gripper
object interfacing with Baxter in ROS2.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# used for working out errors
import traceback

# used for running the main function
import sys

# used for type hinting
from typing import (
    Any,
    List,
    TYPE_CHECKING,
)

# used for ros2 python connection
import rclpy # type: ignore

if TYPE_CHECKING:
    # DigitalIO
    from baxter_int_ros2.baxter_int_ros2 import (
        _TIMER,
        DigitalIO,
        Gripper,
        Topics,
    )
else:
    from baxter_int_ros2 import (
        _TIMER,
        DigitalIO,
        Gripper,
        Topics,
    )


# =============================================================================
# Test 01 - Open and Close Grippers using DigitalIO
# =============================================================================
def t01(
        verbose: int = -1
):
    '''
    Test 01 - Open and Close `Gripper` objects using `DigitalIO`.
    -
    Connects with the 2 `Gripper` objects and open/closes them using the cuff 
    `DigitalIO` objects.

    Parameters
    -
    - verbose : `int`
        - Defaults to `-1`, which means no verbosity. Any value 0 or greater
            corresponds to verbosity with the specified number of tabs used for
            indentation.

    Returns
    -
    None
    '''

    # set verbosity
    V: bool = verbose >= 0
    _t: str = '\t' * verbose
    _sub_v: int = {True: verbose + 1, False: -1}[V]
    if V: 
        print(f'{_t}Gripper Test 01 - Open and Close Grippers using DigitalIO')

    # rclpy initialize
    if V: print(f'| - Initializing RCLPY')
    rclpy.init()

    # creating cameras
    if V: print(f'{_t}| - Creating Grippers and DigitalIOs')
    class Robot_t01():
        def __init__(self) -> None:
            # create grippers
            self.gripper_l = Gripper(Topics.Gripper.LEFT, _sub_v)
            self.gripper_r = Gripper(Topics.Gripper.RIGHT, _sub_v)

            # create buttons
            self.btn_circle_l = DigitalIO(
                Topics.DigitalIO.LEFT_LOWER_BUTTON
            )
            self.btn_circle_r = DigitalIO(
                Topics.DigitalIO.RIGHT_LOWER_BUTTON
            )
            self.btn_dash_l = DigitalIO(
                Topics.DigitalIO.LEFT_UPPER_BUTTON
            )
            self.btn_dash_r = DigitalIO(
                Topics.DigitalIO.RIGHT_UPPER_BUTTON
            )
            self.btn_shoulder_l = DigitalIO(
                Topics.DigitalIO.LEFT_SHOULDER_BUTTON
            )
            self.btn_shoulder_r = DigitalIO(
                Topics.DigitalIO.RIGHT_SHOULDER_BUTTON
            )

            # create button actions
            self.btn_circle_l.state_changed.connect(self.action_btn_circle_l)
            self.btn_circle_r.state_changed.connect(self.action_btn_circle_r)
            self.btn_dash_l.state_changed.connect(self.action_btn_dash_l)
            self.btn_dash_r.state_changed.connect(self.action_btn_dash_r)
            self.btn_shoulder_l.state_changed.connect(
                self.action_btn_shoulder_l
            )
            self.btn_shoulder_r.state_changed.connect(
                self.action_btn_shoulder_r
            )

        # list of nodes being used
        @property
        def nodes(self) -> List[Any]:
            return [
                self.gripper_l,
                self.gripper_r,
                self.btn_circle_l,
                self.btn_circle_r,
                self.btn_dash_l,
                self.btn_dash_r,
                self.btn_shoulder_l,
                self.btn_shoulder_r,
            ]
        
        # button action functions
        def action_btn_circle_l(self, value: bool) -> None:
            if value:
                pos_old: float = self.gripper_l.data_state.position
                pos_new: float = min(100, pos_old + 10)
                if V: 
                    print(
                        f'{_t}\t| - Left Circle - Moving Left Gripper from ' \
                            + f'{pos_old} to {pos_new}'
                    )
                self.gripper_l.set_pos(pos_new) 
        def action_btn_circle_r(self, value: bool) -> None:
            if value:
                pos_old: float = self.gripper_r.data_state.position
                pos_new: float = min(100, pos_old + 10)
                if V: 
                    print(
                        f'{_t}\t| - Right Circle - Moving Right Gripper from' \
                            + f' {pos_old} to {pos_new}'
                    )
                self.gripper_r.set_pos(pos_new) 
        def action_btn_dash_l(self, value: bool) -> None:
            if value:
                pos_old: float = self.gripper_l.data_state.position
                pos_new: float = max(0, pos_old - 10)
                if V: 
                    print(
                        f'{_t}\t| - Left Dash - Moving Left Gripper from ' \
                            + f'{pos_old} to {pos_new}'
                    )
                self.gripper_l.set_pos(pos_new) 
        def action_btn_dash_r(self, value: bool) -> None:
            if value:
                pos_old: float = self.gripper_r.data_state.position
                pos_new: float = max(0, pos_old - 10)
                if V: 
                    print(
                        f'{_t}\t| - Right Dash - Moving Right Gripper from ' \
                            + f'{pos_old} to {pos_new}'
                    )
                self.gripper_r.set_pos(pos_new) 
        def action_btn_shoulder_l(self, value: bool) -> None:
            if value:
                if V: 
                    print(
                        f'{_t}\t| - Left shoulder - Calibrating Left Gripper'
                    )
                self.gripper_l.calibrate()
        def action_btn_shoulder_r(self, value: bool) -> None:
            if V: print(f'{_t}\t| - Right Shoulder Press: {value}')
            if value:
                if V: print(f'{_t}\t\t| - Right Gripper Configure')
                self.gripper_r.configure()
                if V: print(f'{_t}\t\t| - Right Gripper Calibrate')
                self.gripper_r.calibrate()
    r = Robot_t01()

    # spin
    print(f'{_t}| - Spinning')
    executor = rclpy.executors.MultiThreadedExecutor()
    # executor = rclpy.executors.SingleThreadedExecutor()
    for node in r.nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except ConnectionAbortedError:
        print(f'{_t}| - Image Connection Aborted')
    except KeyboardInterrupt:
        print(f'{_t}| - Keyboard Interrupt Occurred')
    except Exception as e:
        print(f'{_t}| - UNKNOWN ERROR: {e}')
        traceback.print_exc()

    # end
    print(f'{_t}| - Destroying Nodes')
    for node in r.nodes:
        node.destroy_node()
    print(f'{_t}| - Shutting Down')
    rclpy.shutdown()
    print(f'{_t}| - Done')


# =============================================================================
# Main Function
# =============================================================================
def main(args: None = None):
    test_num: int
    v: int = -1

    if len(sys.argv) >= 2:
        test_num = int(sys.argv[1])
    else:
        raise ValueError(
            f'test_num (argument 1 value) not specified.'
        )
    if len(sys.argv) >= 3:
        v = int(sys.argv[2])

    if test_num == 1:
        t01(v)
    else:
        raise ValueError(
            f'test_num (argument 1 value) invalid: test_num={repr(test_num)}'
        )

if __name__ == '__main__':
    sys.exit(main())


# =============================================================================
# End of File
# =============================================================================
