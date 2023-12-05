#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter Camera Tester
-
Contains functions which are able to test various parts of the Camera 
object interfacing with Baxter in ROS2.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# used for argument parsing
import argparse

# used for multi-threading
import threading

# used for image processing
import cv2

# used for running the main function
import sys

# used for image processing and numpy arrays
import numpy

# used for type hinting
from typing import (
    Any,
    List,
    TYPE_CHECKING,
)

# used for ros2 python connection
import rclpy # type: ignore
from rclpy.node import Node # type: ignore

if TYPE_CHECKING:
    # DigitalIO
    from baxter_int_ros2.baxter_int_ros2 import (
        _TIMER,
        Camera,
        Client_Camera,
        Services,
        Topics,
    )
else:
    from baxter_int_ros2 import (
        _TIMER,
        Camera,
        Client_Camera,
        Services,
        Topics,
    )


# =============================================================================
# Image Streamer
# =============================================================================
def stream_image(
        args: None = None
) -> None:
    '''
    Image Streamer
    -
    Streams the image data from a particular camera.

    Parameters (Parsed by Environment on `ros2 run ...`)
    -
    - cam_id : `int`
        - `1` = Left Camera.
        - `2` = Right Camera.
        - `3` = Head Camera.
        - All other values are invalid.
    - verbose : `int`
        - Defaults to `-1`, which means no verbosity. Any value 0 or greater
            corresponds to verbosity with the specified number of tabs used for
            indentation.

    Returns
    -
    None
    '''

    cam_id: int = 1
    verbose: int = -1

    if len(sys.argv) >= 2:
        cam_id = int(sys.argv[1])
    if len(sys.argv) >= 3:
        verbose = int(sys.argv[2])

    # set verbosity
    V: bool = verbose >= 0
    _t: str = '\t' * verbose
    _sub_v: int = {True: verbose + 1, False: -1}[V]
    if V: print(f'{_t}Image Streamer')

    # set camera type
    if V: print(f'{_t}| - Setting Camera Type')
    cam_topic: str
    if cam_id == 1:
        cam_topic = Topics.Camera.LEFT
    elif cam_id == 2:
        cam_topic = Topics.Camera.RIGHT
    elif cam_id == 3:
        cam_topic = Topics.Camera.HEAD
    else:
        raise ValueError(
            f'Image Streamer - cam_id = {cam_id}'
        )

    # initialize rclpy
    if V: print(f'{_t}| - Initialize RCLPY')
    rclpy.init()

    # create subscriber
    if V: print(f'{_t}| - Creating Camera Subscriber')
    client = Client_Camera(
        cam_topic,
        Services.Camera.IMAGE_DATA,
        verbose = _sub_v
    )

    # spin
    if V: print(f'{_t}| - Spinning')
    rclpy.spin(client)

    # end
    if V: print(f'{_t}| - Destroying Node')
    client.destroy_node()
    if V: print(f'{_t}| - Shutting Down')
    rclpy.shutdown()
    if V: print(f'{_t}| - Done')


# =============================================================================
# Main Function
# =============================================================================
def main(args: None = None) -> None:
    # runtype: str = sys.argv[1]
    # verbose: int = 0

    # if runtype == 'stream_image':
    #     # stream_image(int(sys.argv[2]), verbose)
    pass

if __name__ == '__main__':
    main()


# =============================================================================
# End of File
# =============================================================================
