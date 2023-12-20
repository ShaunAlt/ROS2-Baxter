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
        print(f'{_t}Gripper Test 01 - Open and Close Grippers using DigitalIO')

    # rclpy initialize
    if V: print(f'| - Initializing RCLPY')
    rclpy.init()

    # creating cameras
    if V: print(f'{_t}| - Creating Grippers and DigitalIOs')
    class Robot_t01():
        def __init__(self) -> None:
            pass

        # list of nodes being used
        @property
        def nodes(self) -> List[Any]:
            return [
            ]
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
