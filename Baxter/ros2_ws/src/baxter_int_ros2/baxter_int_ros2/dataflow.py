#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
ROS2 Baxter Dataflow
-
Contains functions used for controlling the flow of data to and from Baxter in
ROS2.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

from . import (
    # - typing
    Any,
    Callable,
    Dict,
    List,
    Optional,
    TYPE_CHECKING,
    Union,

    # - inspect
    ismethod,

    # - weakref & _weakrefset
    WeakKeyDictionary,
    WeakSet,

    # - rclpy
    rclpy_ok,
    Node,
    Rate,

    # - time
    time,
)


# =============================================================================
# Dataflow Wait
# =============================================================================
def df_wait(
        flag_func: Callable[[], bool],
        node: Node,
        timeout: float = 0,
        raise_err: bool = True,
        ros_rate: int = 100,
        timeout_msg: str = "dataflow wait timeout",
        during_func: Optional[Callable[[], Any]] = None,
        verbose: bool = False
) -> bool:
    '''
    Dataflow Wait
    -
    Waits until either the `flag_func` function returns a `True` value, or the
    timeout expires, and then returns and/or raises an Error as required.

    Parameters
    -
    - flag_func : `Callable[[], bool]`
        - Function used to check whether the waiting period is over. Once this
            function returns a value of `True`, the waiting period is 
            automatically skipped.
    - node : `Node`
        - ROS2 (rclpy) node object which is used for creating the thread rate
            for the wait function.
    - timeout : `float`
        - Number of seconds after which to timeout the flag function and raise
            an error. Defaults to `0` seconds, which means no (infinite)
            timeout.
    - raise_err : `bool`
        - Flag for whether or not to raise an error if the timeout is reached
            before the `flag_func` returns a `True` value. Defaults to `True`,
            which means an error will be raised.
    - ros_rate : `int`
        - The rate at which to check the ROS topics (Hz). Defaults to `100`Hz.
    - timeout_msg : `str`
        - String to print in errors if the timeout is reached. Defaults to
            `"dataflow wait timeout"`.
    - during_func : `Callable[[], ...] | None`
        - Optional function to run whilst waiting for the main `flag_func`
            to return a `True` value. Defaults to `None`, which means nothing
            will run.
    - verbose : `bool`
        - Flag for whether or not to run the function with verbose output.

    Returns
    -
    - `bool`
        - Whether or not the `flag_func` returned a `True` value.
    '''

    if verbose: print('Running DF-Wait')

    # validate that the `ros_rate` is an integer
    if verbose: print('| - Validating ROS_RATE')
    if (
            (not isinstance(ros_rate, int)) \
            or (ros_rate <= 0)
    ):
        raise ValueError(
            'baxter_int_ros2.dataflow.df_wait - ros_rate is invalid: ' \
                + f'ros_rate = {ros_rate}, type = {type(ros_rate)}'
        )
    
    # validate that the `timeout` is a positive float/int
    if verbose: print('| - Validating TIMEOUT')
    if (
            (
                (not isinstance(timeout, int)) \
                and (not isinstance(timeout, float))
            ) \
            or (timeout < 0)
    ):
        raise ValueError(
            'baxter_int_ros2.dataflow.df_wait - timeout is invalid: timeout ' \
                + f'= {timeout}, type = {type(timeout)}'
        )

    # initialize variables
    if verbose: print('| - Initializing Variables')
    end_time: float = time.time() + timeout
    rate: Rate = node.create_rate(ros_rate)
    skip_timeout: bool = timeout == 0
    i: int = 0
    if verbose:
        print(
            f'\t| - end_time = {end_time}\n\t| - rate = {rate}\n\t ' \
                + f'skip_timeout = {skip_timeout}'
        )

    if verbose: print('| - Running Timeout Loop')
    while (not flag_func()):
        if verbose: print(f'\t| - Loop Iter. {i}')
        # raise ERR if ROS2 shuts down
        if verbose: print(f'\t\t| - Check ROS2 Shutdown')
        if (not rclpy_ok()):
            if raise_err:
                raise RuntimeError(
                    'baxter_int_ros2.dataflow.df_wait - ROS2 Shutdown'
                )
            return False
        # raise ERR if TIMEOUT
        if verbose: print(f'\t\t| - Check timeout')
        elif (
                (time.time() >= end_time)
                and (not skip_timeout) \
        ):
            if raise_err:
                raise TimeoutError(
                    'baxter_int_ros2.dataflow.df_wait - Message = ' \
                        + f'{timeout_msg}'
                )
            return False
        
        # run during-func if defined
        if callable(during_func):
            if verbose: print('\t\t| - Running DURING_FUNC()')
            during_func()
        
        # ROS2 sleep for the required amount
        if verbose: print(f'\t\t| - Incrementing and Sleeping')
        i += 1
        rate.sleep()

    return True


# =============================================================================
# Signal Object
# =============================================================================
class Signal():
    '''
    Signal Object
    -
    TODO: The purpose of this object has not yet been worked out, it was taken
    from ROS1 however the use is currently unknown.
    '''

    def __init__(self) -> None:
        self._functions: WeakSet = WeakSet()
        self._methods: WeakKeyDictionary = WeakKeyDictionary()

    def __call__(self, *args: Any, **kwargs: Any) -> None:
        for f in self._functions:
            f(*args, **kwargs)
        for obj, funcs in self._methods.items():
            for f in funcs:
                f(obj, *args, **kwargs)

    def connect(self, slot: Callable) -> None:
        if ismethod(slot):
            if not slot.__self__ in self._methods:
                self._methods[slot.__self__] = set()
            self._methods[slot.__self__].add(slot.__func__)
        else:
            self._functions.add(slot)

    def disconnect(self, slot: Callable) -> None:
        if ismethod(slot):
            if slot.__self__ in self._methods:
                self._methods[slot.__self__].remove(slot.__func__)
        else:
            if slot in self._functions:
                self._functions.remove(slot)

# def main(args=None):
#     rclpy.init(args=args)
#     minimal_subscriber = MinimalSubscriber()
#     rclpy.spin(minimal_subscriber)
#     minimal_subscriber.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# =============================================================================
# End of File
# =============================================================================
