#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter DigitalIO Control Tester
-
Contains functions which are able to test various parts of the DigitalIO 
object interfacing with Baxter in ROS2.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# used for type hinting
from typing import (
    Any,
    List,
    TYPE_CHECKING,
)

# used for runtime arguments
import sys

# used for ros2 python connection
import rclpy # type: ignore

if TYPE_CHECKING:
    # DigitalIO
    from baxter_int_ros2.baxter_int_ros2 import (
        DigitalIO,
        Topics,
    )
else:
    from baxter_int_ros2 import (
        DigitalIO,
        Topics,
    )


# =============================================================================
# Test 01 - Connect with Gripper DigitalIOs
# =============================================================================
def test01_connect_gripper_digitalios() -> None:
    '''
    Test 1 - Connect with Gripper DigitalIOs
    -
    Connects with the 6 `DigitalIO` objects on the 2 grippers of Baxter, and
    prints the data out.

    Parameters
    -
    None

    Returns
    -
    None
    '''

    class Grippers():
        def __init__(self) -> None:
            self.l_circle = DigitalIO(
                Topics.DigitalIO.LEFT_LOWER_BUTTON,
                verbose = True
            )
            self.l_line = DigitalIO(Topics.DigitalIO.LEFT_UPPER_BUTTON)
            self.l_cuff = DigitalIO(Topics.DigitalIO.LEFT_LOWER_CUFF)
            self.r_circle = DigitalIO(Topics.DigitalIO.RIGHT_LOWER_BUTTON)
            self.r_line = DigitalIO(Topics.DigitalIO.RIGHT_UPPER_BUTTON)
            self.r_cuff = DigitalIO(Topics.DigitalIO.RIGHT_LOWER_CUFF)

            self.l_circle.state_changed.connect(self.action_l_circle)
            self.l_line.state_changed.connect(self.action_l_line)
            self.l_cuff.state_changed.connect(self.action_l_cuff)
            self.r_circle.state_changed.connect(self.action_r_circle)
            self.r_line.state_changed.connect(self.action_r_line)
            self.r_cuff.state_changed.connect(self.action_r_cuff)

        def action_l_circle(self, value: bool) -> None:
            self.l_circle.get_logger().info(
                f'{self.l_circle}: Value = {value}'
            )

        def action_l_line(self, value: bool) -> None:
            self.l_line.get_logger().info(
                f'{self.l_line}: Value = {value}'
            )

        def action_l_cuff(self, value: bool) -> None:
            self.l_cuff.get_logger().info(
                f'{self.l_cuff}: Value = {value}'
            )

        def action_r_circle(self, value: bool) -> None:
            self.r_circle.get_logger().info(
                f'{self.r_circle}: Value = {value}'
            )

        def action_r_line(self, value: bool) -> None:
            self.r_line.get_logger().info(
                f'{self.r_line}: Value = {value}'
            )

        def action_r_cuff(self, value: bool) -> None:
            self.r_cuff.get_logger().info(
                f'{self.r_cuff}: Value = {value}'
            )
    
    print('Initializing RCLPY')
    rclpy.init()
    print('Getting Grippers')
    g = Grippers()
    print('Spinning')
    executor = rclpy.executors.MultiThreadedExecutor()
    for node in [g.l_circle, g.l_line, g.l_cuff, g.r_circle, g.r_line, g.r_cuff]:
        executor.add_node(node)
    executor.spin()
    # rclpy.spin()
    print('Shutting Down')
    rclpy.shutdown()
    print('Done')


# =============================================================================
# Test 02 - Button IOs with Lights
# =============================================================================
def test02_buttons_with_lights(
        verbose: int = -1
) -> None:
    '''
    Test 02 - Button IOs with Lights
    -
    Connects with all of the Baxter buttons that have lights around/next to
    them and toggles the lights whenever the buttons are pressed.

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
    print(f'{_t}Digital IO Test 02 - Button IOs with Lights')

    # initialize rclpy
    print(f'{_t}| - Initialize RCLPY')
    rclpy.init()

    # create DigitalIO objects
    class Objects():
        def __init__(self) -> None:
            self.btn_r_ok = DigitalIO(Topics.DigitalIO.RIGHT_BUTTON_OK)
            self.btn_r_back = DigitalIO(Topics.DigitalIO.RIGHT_BUTTON_BACK)
            self.btn_r_show = DigitalIO(Topics.DigitalIO.RIGHT_BUTTON_SHOW)
            self.btn_l_ok = DigitalIO(Topics.DigitalIO.LEFT_BUTTON_OK)
            self.btn_l_back = DigitalIO(Topics.DigitalIO.LEFT_BUTTON_BACK)
            self.btn_l_show = DigitalIO(Topics.DigitalIO.LEFT_BUTTON_SHOW)
            self.light_r_ok = DigitalIO(Topics.DigitalIO.RIGHT_INNER_LIGHT)
            self.light_r_back = DigitalIO(Topics.DigitalIO.RIGHT_OUTER_LIGHT)
            self.light_r_show = DigitalIO(Topics.DigitalIO.RIGHT_BLUE_LIGHT)
            self.light_l_ok = DigitalIO(Topics.DigitalIO.LEFT_INNER_LIGHT)
            self.light_l_back = DigitalIO(Topics.DigitalIO.LEFT_OUTER_LIGHT)
            # self.light_l_show = DigitalIO(Topics.DigitalIO.LEFT_BLUE_LIGHT)

            self.btn_r_ok.state_changed.connect(self._act_btn_r_ok)
            self.btn_r_back.state_changed.connect(self._act_btn_r_back)
            self.btn_r_show.state_changed.connect(self._act_btn_r_show)
            self.btn_l_ok.state_changed.connect(self._act_btn_l_ok)
            self.btn_l_back.state_changed.connect(self._act_btn_l_back)
            self.btn_l_show.state_changed.connect(self._act_btn_l_show)

        def _act_btn_r_ok(self, value: bool) -> None:
            if V: print(f'{_t}\t| - Button Right OK Press: {value}')
            self.light_r_ok.state = value

        def _act_btn_r_back(self, value: bool) -> None:
            if V: print(f'{_t}\t| - Button Right Back Press: {value}')
            self.light_r_back.state = value

        def _act_btn_r_show(self, value: bool) -> None:
            if V: print(f'{_t}\t| - Button Right Show Press: {value}')
            self.light_r_show.state = value

        def _act_btn_l_ok(self, value: bool) -> None:
            if V: print(f'{_t}\t| - Button Left OK Press: {value}')
            self.light_l_ok.state = value

        def _act_btn_l_back(self, value: bool) -> None:
            if V: print(f'{_t}\t| - Button Left Back Press: {value}')
            self.light_l_back.state = value

        def _act_btn_l_show(self, value: bool) -> None:
            if V: print(f'{_t}\t| - Button Left Show Press: {value}')
            # self.light_l_show.state = value

        def get_nodes(self) -> List[DigitalIO]:
            return [
                self.btn_r_ok,
                self.btn_r_back,
                self.btn_r_show,
                self.btn_l_ok,
                self.btn_l_back,
                self.btn_l_show,
                self.light_r_ok,
                self.light_r_back,
                self.light_r_show,
                self.light_l_ok,
                self.light_l_back,
                # self.light_l_show,
            ]
        
    # create nodes
    print(f'{_t}| - Creating Nodes')
    _objs = Objects()

    # spin
    print(f'{_t}| - Spinning')
    executor = rclpy.executors.MultiThreadedExecutor()
    for node in _objs.get_nodes():
        executor.add_node(node)
    try:
        executor.spin()
    except:
        print(f'\n')

    # end
    print(f'{_t}| - Destroying Nodes')
    for node in _objs.get_nodes():
        node.destroy_node()
    print(f'{_t}| - Shutting Down')
    rclpy.shutdown()
    print(f'{_t}| - Done')


# =============================================================================
# Main Function
# =============================================================================
def main() -> None:
    test_num: int
    v: int = 0

    if len(sys.argv) >= 2:
        test_num = int(sys.argv[1])
    else:
        raise RuntimeError(
            f'Main Positional Argument <test_num> not specified'
        )
    
    if len(sys.argv) >= 3:
        v = int(sys.argv[2])

    if test_num == 1:
        test01_connect_gripper_digitalios()
    elif test_num == 2:
        test02_buttons_with_lights(v)
    else:
        raise ValueError(
            f'test_num invalid: test_num={repr(test_num)}'
        )

if __name__ == '__main__':
    main()


# =============================================================================
# End of File
# =============================================================================
