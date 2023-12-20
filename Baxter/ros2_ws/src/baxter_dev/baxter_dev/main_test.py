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
    Optional,
    Tuple,
    Type,
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
        ROS2_Node,
        Topics,
    )
else:
    from baxter_int_ros2 import (
        _TIMER,
        DigitalIO,
        Gripper,
        Limb,
        ROS2_Node,
        Topics,
    )


# =============================================================================
# Test Runner
# =============================================================================
_TEST_RETURN = Tuple[str, Type[ROS2_Node]] # return-type of all test functions
def run_test(
        test_num: int,
        verbose: int = -1
) -> None:
    '''
    Test Runner
    -
    Runs a particular test.

    Parameters
    -
    - test_num : `int`
        - Which test number to complete.
    - verbose : `int`
        - Defaults to `-1`, which means no verbosity. Any value 0 or greater
            corresponds to verbosity with the specified number of tabs used for
            indentation.

    Returns
    -
    None
    '''

    print(f'Running Main Test {test_num:>02d} with Verbosity {verbose}')

    # get test data
    print('| - Creating Test Nodes')
    txt: str
    _r: Any
    if test_num == 1:
        txt, _r = t01()
    elif test_num == 2:
        txt, _r = t02()
    else:
        raise ValueError(
            f'test_num (argument 1 value) invalid: test_num={repr(test_num)}'
        )
    print(f'| - Running Test: {txt}')

    # rclpy initialize
    print(f'| - Initializing RCLPY')
    rclpy.init()

    # create nodes
    print('| - Creating Nodes')
    r = _r(verbose)

    # spin
    print('| - Spinning')
    executor = rclpy.executors.MultiThreadedExecutor()
    # executor = rclpy.executors.SingleThreadedExecutor()
    for node in r.nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except ConnectionAbortedError:
        print('| - Image Connection Aborted')
    except KeyboardInterrupt:
        print('| - Keyboard Interrupt Occurred')
    except Exception as e:
        print(f'| - UNKNOWN ERROR: {e}')
        traceback.print_exc()

    # end
    print('| - Destroying Nodes')
    for node in r.nodes:
        node.destroy_node()
    print('| - Shutting Down')
    rclpy.shutdown()
    print('| - Done')


# =============================================================================
# Test 01
# =============================================================================
def t01() -> _TEST_RETURN:
    '''
    Test 01
    -
    Read the `Limb` positions, velocities, and efforts using `DigitalIO` button
    reads.
    '''

    # define print text
    txt: str = (
        'Main Test 01: Read Limb positions, velocities, and efforts using ' \
        + 'DigitalIO button reads.'
    )    

    # define robot
    class Robot(ROS2_Node):
        def __init__(
                self,
                verbose: int = -1
        ) -> None:
            # setup node
            super().__init__('Baxter_Robot_Test01', verbose)

            # create limbs
            self.limb_L = Limb(
                Topics.Limb.LEFT, 
                {True: self._verbose_sub, False: 0}[self._V]
            )
            self.limb_R = Limb(
                Topics.Limb.RIGHT, 
                {True: self._verbose_sub, False: 0}[self._V]
            )

            # create buttons for reading
            self.btn_back_L = DigitalIO(
                Topics.DigitalIO.LEFT_BUTTON_BACK,
                self._verbose_sub
            )
            self.btn_ok_L = DigitalIO(
                Topics.DigitalIO.LEFT_BUTTON_OK,
                self._verbose_sub
            )
            self.btn_shoulder_L = DigitalIO(
                Topics.DigitalIO.LEFT_SHOULDER_BUTTON,
                self._verbose_sub
            )
            self.btn_show_L = DigitalIO(
                Topics.DigitalIO.LEFT_BUTTON_SHOW,
                self._verbose_sub
            )
            self.btn_back_R = DigitalIO(
                Topics.DigitalIO.RIGHT_BUTTON_BACK,
                self._verbose_sub
            )
            self.btn_ok_R = DigitalIO(
                Topics.DigitalIO.RIGHT_BUTTON_OK,
                self._verbose_sub
            )
            self.btn_shoulder_R = DigitalIO(
                Topics.DigitalIO.RIGHT_SHOULDER_BUTTON,
                self._verbose_sub
            )
            self.btn_show_R = DigitalIO(
                Topics.DigitalIO.RIGHT_BUTTON_SHOW,
                self._verbose_sub
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
            self.btn_back_L.log(f'| - Display Left Positions: {value}')
            if value:
                self.limb_L.log(
                    f'| - Left Positions: {self.limb_L.data_positions}'
                )
        def disp_vel_L(self, value: bool) -> None:
            self.btn_ok_L.log(f'| - Display Left Velocites: {value}')
            if value:
                self.limb_L.log(
                    f'| - Left Velocites: {self.limb_L.data_velocities}'
                )
        def disp_tor_L(self, value: bool) -> None:
            self.btn_show_L.log(f'| - Display Left Torques: {value}')
            if value:
                self.limb_L.log(
                    f'| - Left Torques: {self.limb_L.data_torques}'
                )
        def disp_all_L(self, value: bool) -> None:
            self.btn_shoulder_L.log(f'| - Display Left All Data: {value}')
            if value:
                self.limb_L.log(
                    f'| - Left Data:\n' \
                    + f'\t| - Positions: {self.limb_L.data_positions}\n' \
                    + f'\t| - Velocities: {self.limb_L.data_velocities}' \
                    + f'\n\t| - Torques: {self.limb_L.data_torques}' \
                )
        def disp_pos_R(self, value: bool) -> None:
            self.btn_back_R.log(f'| - Display Right Positions: {value}')
            if value:
                self.limb_R.log(
                    f'| - Right Positions: {self.limb_R.data_positions}'
                )
        def disp_vel_R(self, value: bool) -> None:
            self.btn_ok_R.log(f'| - Display Right Velocites: {value}')
            if value:
                self.limb_R.log(
                    f'| - Right Velocites: {self.limb_R.data_velocities}'
                )
        def disp_tor_R(self, value: bool) -> None:
            self.btn_show_R.log(f'| - Display Right Torques: {value}')
            if value:
                self.limb_R.log(
                    f'| - Right Torques: {self.limb_R.data_torques}'
                )
        def disp_all_R(self, value: bool) -> None:
            self.btn_shoulder_R.log(f'| - Display Right All Data: {value}')
            if value:
                self.limb_R.log(
                    f'| - Right Data:\n' \
                    + f'\t| - Positions: {self.limb_R.data_positions}\n' \
                    + f'\t| - Velocities: {self.limb_R.data_velocities}' \
                    + f'\n\t| - Torques: {self.limb_R.data_torques}' \
                )
    
    return txt, Robot


# =============================================================================
# Test 02
# =============================================================================
def t02() -> _TEST_RETURN:
    '''
    Test 02
    -
    Read and save `Limb` positions on 1 button press, and then move to the
    saved positions on different button press.
    '''

    # define print text
    txt: str = (
        'Main Test 02: Read/Save Limb Positions, and then move back to them ' \
        + 'DigitalIO button presses.'
    )

    # define robot
    class Robot(ROS2_Node):
        def __init__(
                self,
                verbose: int = -1
        ) -> None:
            # setup node
            super().__init__('Baxter_Robot_Test02', verbose)

            # create limbs
            self.limb_L = Limb(
                Topics.Limb.LEFT, 
                {True: self._verbose_sub, False: 0}[self._V]
            )
            self.limb_R = Limb(
                Topics.Limb.RIGHT,
                {True: self._verbose_sub, False: 0}[self._V]
            )

            # create buttons for reading
            self.btn_back_L = DigitalIO(
                Topics.DigitalIO.LEFT_BUTTON_BACK,
                self._verbose_sub
            )
            self.btn_ok_L = DigitalIO(
                Topics.DigitalIO.LEFT_BUTTON_OK,
                self._verbose_sub
            )
            self.btn_back_R = DigitalIO(
                Topics.DigitalIO.RIGHT_BUTTON_BACK,
                self._verbose_sub
            )
            self.btn_ok_R = DigitalIO(
                Topics.DigitalIO.RIGHT_BUTTON_OK,
                self._verbose_sub
            )

            # storing the positions of the arms
            self.pos_L = Robot.Limb_Positions()
            self.pos_R = Robot.Limb_Positions()

            # create button action event listeners
            self.btn_back_L.state_changed.connect(self.move_L)
            self.btn_ok_L.state_changed.connect(self.save_L)
            self.btn_back_R.state_changed.connect(self.move_R)
            self.btn_ok_R.state_changed.connect(self.save_R)

        # list of nodes being used
        @property
        def nodes(self) -> List[Any]:
            return [
                self.limb_L,
                self.limb_R,
                self.btn_back_L,
                self.btn_ok_L,
                self.btn_back_R,
                self.btn_ok_R,
            ]

        # button actions
        def save_L(self, value: bool) -> None:
            self.btn_ok_L.log(f'| - Save Left Position: {value}')
            if value:
                if None in [
                        self.limb_L.data_joint_e0,
                        self.limb_L.data_joint_e1,
                        self.limb_L.data_joint_s0,
                        self.limb_L.data_joint_s1,
                        self.limb_L.data_joint_w0,
                        self.limb_L.data_joint_w1,
                        self.limb_L.data_joint_w2,
                ]:
                    raise RuntimeError(
                        'Tried to save Left Position but no positions ' \
                        + 'currently read by subscriber'
                    )
                self.pos_L.e0 = self.limb_L.data_joint_e0[0]
                self.pos_L.e1 = self.limb_L.data_joint_e1[0]
                self.pos_L.s0 = self.limb_L.data_joint_s0[0]
                self.pos_L.s1 = self.limb_L.data_joint_s1[0]
                self.pos_L.w0 = self.limb_L.data_joint_w0[0]
                self.pos_L.w1 = self.limb_L.data_joint_w1[0]
                self.pos_L.w2 = self.limb_L.data_joint_w2[0]
                self.limb_L.log(f'| - Saved Left Positions: {self.pos_L}')
        def move_L(self, value: bool) -> None:
            self.btn_back_L.log(f'| - Move Left Position: {value}')
            if value:
                if self.pos_L.contains_null():
                    raise RuntimeError(
                        'Tried to Move Left Position but no position saved'
                    )
                self.limb_L.set_positions(
                    timeout = 5.0,
                    e0 = self.pos_L.e0,
                    e1 = self.pos_L.e1,
                    s0 = self.pos_L.s0,
                    s1 = self.pos_L.s1,
                    w0 = self.pos_L.w0,
                    w1 = self.pos_L.w1,
                    w2 = self.pos_L.w2
                )
                self.limb_L.log(f'| - Moved to Left Position: {self.pos_L}')
        def save_R(self, value: bool) -> None:
            self.btn_ok_R.log(f'| - Save Right Position: {value}')
            if value:
                if None in [
                        self.limb_R.data_joint_e0,
                        self.limb_R.data_joint_e1,
                        self.limb_R.data_joint_s0,
                        self.limb_R.data_joint_s1,
                        self.limb_R.data_joint_w0,
                        self.limb_R.data_joint_w1,
                        self.limb_R.data_joint_w2,
                ]:
                    raise RuntimeError(
                        'Tried to save Right Position but no positions ' \
                        + 'currently read by subscriber'
                    )
                self.pos_R.e0 = self.limb_R.data_joint_e0[0]
                self.pos_R.e1 = self.limb_R.data_joint_e1[0]
                self.pos_R.s0 = self.limb_R.data_joint_s0[0]
                self.pos_R.s1 = self.limb_R.data_joint_s1[0]
                self.pos_R.w0 = self.limb_R.data_joint_w0[0]
                self.pos_R.w1 = self.limb_R.data_joint_w1[0]
                self.pos_R.w2 = self.limb_R.data_joint_w2[0]
                self.limb_R.log(f'| - Saved Right Positions: {self.pos_R}')
        def move_R(self, value: bool) -> None:
            self.btn_back_R.log(f'| - Move Right Position: {value}')
            if value:
                if self.pos_R.contains_null():
                    raise RuntimeError(
                        'Tried to Move Right Position but no position saved'
                    )
                self.limb_R.set_positions(
                    timeout = 5.0,
                    e0 = self.pos_R.e0,
                    e1 = self.pos_R.e1,
                    s0 = self.pos_R.s0,
                    s1 = self.pos_R.s1,
                    w0 = self.pos_R.w0,
                    w1 = self.pos_R.w1,
                    w2 = self.pos_R.w2
                )
                self.limb_R.log(f'| - Moved to Right Position: {self.pos_R}')

        class Limb_Positions():
            def __init__(self) -> None:
                self.e0: Optional[float] = None
                self.e1: Optional[float] = None
                self.s0: Optional[float] = None
                self.s1: Optional[float] = None
                self.w0: Optional[float] = None
                self.w1: Optional[float] = None
                self.w2: Optional[float] = None
            def __str__(self) -> str:
                if self.contains_null():
                    return 'Positions: Not Set'
                return (
                    f'Positions:\n\tShoulder:\n\t\tS0: {self.s0:.3f}\n\t\t' \
                    + f'S1: {self.s1:.3f}\n\tElbow:\n\t\tE0: {self.e0:.3f}\n' \
                    + f'\t\tE1: {self.e1:.3f}\n\tWrist:\n\t\tW0: ' \
                    + f'{self.w0:.3f}\n\t\tW1: {self.w1:.3f}\n\t\tW2: ' \
                    + f'{self.w2:.3f}'
                )
            def contains_null(self) -> bool:
                return None in [
                    self.e0,
                    self.e1,
                    self.s0,
                    self.s1,
                    self.w0,
                    self.w1,
                    self.w2,
                ]

    return txt, Robot


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

    run_test(test_num, v)

if __name__ == '__main__':
    sys.exit(main())


# =============================================================================
# End of File
# =============================================================================
