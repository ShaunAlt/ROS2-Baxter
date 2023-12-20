#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter Main Functionality Tester
-
Contains functions which are able to test various parts of the entire Baxter
interface and functionality with ROS2.
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
        Limb,
        Topics,
    )
else:
    from baxter_int_ros2 import (
        _TIMER,
        DigitalIO,
        Gripper,
        Limb,
        Topics,
    )


# =============================================================================
# Test 01
# =============================================================================
def t01(
        verbose: int = -1
):
    '''
    Test 01
    -
    Read the `Limb` positions, velocities, and efforts using `DigitalIO` button
    reads.

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
        print(
            f'{_t}Main Test 01:\n\t{_t}Read the `Limb` positions, ' \
            + 'velocities, and efforts using `DigitalIO` button reads.'
        )

    # rclpy initialize
    if V: print(f'| - Initializing RCLPY')
    rclpy.init()

    # creating cameras
    if V: print(f'{_t}| - Creating Grippers and DigitalIOs')
    class Robot():
        def __init__(self) -> None:
            # create limbs
            self.limb_L = Limb(Topics.Limb.LEFT, _sub_v)
            self.limb_R = Limb(Topics.Limb.RIGHT, _sub_v)

            # create buttons for reading
            self.btn_back_L = DigitalIO(
                Topics.DigitalIO.LEFT_BUTTON_BACK
            )
            self.btn_ok_L = DigitalIO(
                Topics.DigitalIO.LEFT_BUTTON_OK
            )
            self.btn_shoulder_L = DigitalIO(
                Topics.DigitalIO.LEFT_SHOULDER_BUTTON
            )
            self.btn_show_L = DigitalIO(
                Topics.DigitalIO.LEFT_BUTTON_SHOW
            )
            self.btn_back_R = DigitalIO(
                Topics.DigitalIO.RIGHT_BUTTON_BACK
            )
            self.btn_ok_R = DigitalIO(
                Topics.DigitalIO.RIGHT_BUTTON_OK
            )
            self.btn_shoulder_R = DigitalIO(
                Topics.DigitalIO.RIGHT_SHOULDER_BUTTON
            )
            self.btn_show_R = DigitalIO(
                Topics.DigitalIO.RIGHT_BUTTON_SHOW
            )

            # creating button action event listeners
            self.btn_back_L.state_changed.connect(self.disp_pos_L)
            self.btn_ok_L.state_changed.connect(self.disp_vel_L)
            self.btn_show_L.state_changed.connect(self.disp_tor_L)
            self.btn_shoulder_L.state_changed.connect(self.disp_all_L)
            self.btn_back_R.state_changed.connect(self.disp_pos_R)
            self.btn_ok_R.state_changed.connect(self.disp_vel_R)
            self.btn_show_R.state_changed.connect(self.disp_tor_R)
            self.btn_shoulder_R.state_changed.connect(self.disp_all_R)

        # list of nodes being used
        @property
        def nodes(self) -> List[Any]:
            return [
                self.limb_L,
                self.limb_R,
                self.btn_back_L,
                self.btn_ok_L,
                self.btn_shoulder_L,
                self.btn_show_L,
                self.btn_ok_R,
                self.btn_back_R,
                self.btn_shoulder_R,
                self.btn_show_R,
            ]
        
        # button actions
        def disp_pos_L(self, value: bool) -> None:
            if V: print(f'{_t}| - Display Left Positions: {value}')
            if value:
                print(f'{_t}| - Left Positions: {self.limb_L.data_positions}')
        def disp_vel_L(self, value: bool) -> None:
            if V: print(f'{_t}| - Display Left Velocites: {value}')
            if value:
                print(f'{_t}| - Left Velocites: {self.limb_L.data_velocities}')
        def disp_tor_L(self, value: bool) -> None:
            if V: print(f'{_t}| - Display Left Torques: {value}')
            if value:
                print(f'{_t}| - Left Torques: {self.limb_L.data_torques}')
        def disp_all_L(self, value: bool) -> None:
            if V: print(f'{_t}| - Display Left All Data: {value}')
            if value:
                print(
                    f'{_t}| - Left Data:\n' \
                    + f'\t{_t}| - Positions: {self.limb_L.data_positions}\n' \
                    + f'\t{_t}| - Velocities: {self.limb_L.data_velocities}' \
                    + f'\n\t{_t}| - Torques: {self.limb_L.data_torques}' \
                )
        def disp_pos_R(self, value: bool) -> None:
            if V: print(f'{_t}| - Display Right Positions: {value}')
            if value:
                print(f'{_t}| - Right Positions: {self.limb_R.data_positions}')
        def disp_vel_R(self, value: bool) -> None:
            if V: print(f'{_t}| - Display Right Velocites: {value}')
            if value:
                print(f'{_t}| - Right Velocites: {self.limb_R.data_velocities}')
        def disp_tor_R(self, value: bool) -> None:
            if V: print(f'{_t}| - Display Right Torques: {value}')
            if value:
                print(f'{_t}| - Right Torques: {self.limb_R.data_torques}')
        def disp_all_R(self, value: bool) -> None:
            if V: print(f'{_t}| - Display Right All Data: {value}')
            if value:
                print(
                    f'{_t}| - Right Data:\n' \
                    + f'\t{_t}| - Positions: {self.limb_R.data_positions}\n' \
                    + f'\t{_t}| - Velocities: {self.limb_R.data_velocities}' \
                    + f'\n\t{_t}| - Torques: {self.limb_R.data_torques}' \
                )
    r = Robot()

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
