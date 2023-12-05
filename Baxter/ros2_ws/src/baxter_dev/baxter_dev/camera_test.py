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
        MSG_Image,
        msgImage,
        Services,
        Topics,
    )
else:
    from baxter_int_ros2 import (
        _TIMER,
        Camera,
        Client_Camera,
        MSG_Image,
        msgImage,
        Services,
        Topics,
    )


# =============================================================================
# Test 01 - Read Camera Data
# =============================================================================
def test01_read_images(
        verbose: int = -1
):
    '''
    Test 1 - Read Camera Data
    -
    Connects with the 3 `Camera` objects on Baxter and continuously prints out
    various bits of data from each.

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
    if V: print(f'{_t}Camera Test 1 - Read Image Data')

    # rclpy initialize
    if V: print(f'| - Initializing RCLPY')
    rclpy.init()

    # creating cameras
    if V: print(f'{_t}| - Creating Cameras')
    cam_head = Camera(
        Topics.Camera.HEAD,
        verbose = _sub_v,
        print_image_timer = 1.0
    )
    cam_left = Camera(
        Topics.Camera.LEFT,
        verbose = _sub_v,
        print_image_timer = 1.0
    )
    cam_right = Camera(
        Topics.Camera.RIGHT,
        verbose = _sub_v,
        print_image_timer = 1.0
    )
    all_nodes: List[Node] = [
        cam_head,
        cam_left,
        cam_right,
    ]

    # spinning rclpy
    if V: print(f'| - Creating Executors')
    _exec = rclpy.executors.MultiThreadedExecutor()
    for node in all_nodes:
        _exec.add_node(node)
    if V: print(f'| - Spinning')
    _exec.spin()

    # shutting down
    if V: print(f'| - Destroying Nodes')
    for node in all_nodes:
        node.destroy_node()
    if V: print(f'| - Shutting Down')
    rclpy.shutdown()
    if V: print(f'| - Done')


# =============================================================================
# Test 02 - Display Camera Data
# =============================================================================
def test02_display_images(
        verbose: int = -1
):
    '''
    Test 2 - Display Camera Data
    -
    Connects with the 3 `Camera` objects on Baxter and continuously updates
    OpenCV images which are showing what the camera data is showing.

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
    if V: print(f'{_t}Camera Test 2 - Display Image Data')

    # rclpy initialize
    if V: print(f'| - Initializing RCLPY')
    rclpy.init()

    # creating cameras
    if V: print(f'{_t}| - Creating Cameras')
    cam_head = Camera(
        Topics.Camera.HEAD,
        verbose = _sub_v,
        image_display_rt = True
    )
    cam_left = Camera(
        Topics.Camera.LEFT,
        verbose = _sub_v,
        image_display_rt = True
    )
    cam_right = Camera(
        Topics.Camera.RIGHT,
        verbose = _sub_v,
        image_display_rt = True
    )
    all_nodes: List[Node] = [
        cam_head,
        cam_left,
        cam_right,
    ]

    # spinning rclpy
    if V: print(f'| - Creating Executors')
    _exec = rclpy.executors.MultiThreadedExecutor()
    for node in all_nodes:
        _exec.add_node(node)
    if V: print(f'| - Creating Multi-Thread')
    _thread_spin = threading.Thread(
        target = _exec.spin, 
        daemon = True
    )
    # _thread_cv2 = threading.Thread(
    #     target = cv2.waitKey, 
    #     args = (0, ), 
    #     daemon = True
    # )
    if V: print(f'| - Spinning')
    _thread_spin.start()
    # if V: print(f'| - CV2 WaitKey')
    # _thread_cv2.start()

    if V: print(f'| - Press CTRL+C to Finish: ', end = '')
    while True:
        try:
            pass
        except KeyboardInterrupt:
            break

    # shutting down
    if V: print(f'| - Destroying Nodes')
    for node in all_nodes:
        node.destroy_node()
    if V: print(f'| - Shutting Down')
    rclpy.shutdown()
    if V: print(f'| - Done')
    return 0


# =============================================================================
# Test 03 - Display Single Camera Data
# =============================================================================
def test03_display_image_singular(
        verbose: int = -1
) -> None:
    '''
    Test 3 - Display Single Camera Data
    -
    Connects with the left `Camera` object on Baxter and continuously updates
    OpenCV image output showing what the camera is showing.

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
    if V: print(f'{_t}Camera Test 3 - Display Single Camera Data')

    # create camera data subscriber
    if V: print(f'{_t}| - Creating Camera Subscriber Class')
    class Camera_Subscriber(Node):
        def __init__(self) -> None:
            super().__init__('camera_subscriber')
            self.subscription = self.create_subscription(
                MSG_Image,
                '/cameras/left_hand_camera/image',
                self.listener_callback,
                10
            )

        def listener_callback(self, _msg: msgImage) -> None:
            if V: print(f'{_t}| - Camera Data Read')
            # convert message data type
            if V: print(f'{_t}\t| - Converting Message Data to Custom Type')
            msg = MSG_Image.from_msg(_msg)

            # convert to numpy array
            if V: print(f'{_t}\t| - Converting Image to NumPy Array')
            img_arr = numpy.array(msg.data, dtype = numpy.uint8)
            if V: print(f'{_t}\t\t| - Image 1D: Shape = {img_arr.shape}')

            # convert to 3D array
            if V: print(f'{_t}\t| - Converting 1D to 3D')
            img = numpy.reshape(
                img_arr,
                (
                    msg.height,
                    msg.width,
                    msg.channels,
                )
            )
            if V: print(f'{_t}\t\t| - Image 3D: Shape = {img_arr.shape}')

            # convert colour
            if V: print(f'{_t}\t| - Converting Image Colour')
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            if V: 
                print(
                    f'{_t}\t\t| - Image 3D Colour Corrected: Shape = ' \
                        + f'{img_arr.shape}'
                )
            
            # display image
            if V: print(f'{_t}\t| - Displaying Image')
            cv2.imshow(f'Camera Test 3 Image', img)
            cv2.waitKey(1)

    # initialize rclpy
    if V: print(f'{_t}| - Initialize RCLPY')
    rclpy.init()

    # create subscriber
    if V: print(f'{_t}| - Creating Camera Subscriber')
    sub = Camera_Subscriber()

    # spin
    if V: print(f'{_t}| - Spinning')
    rclpy.spin(sub)

    # end
    if V: print(f'{_t}| - Destroying Node')
    sub.destroy_node()
    if V: print(f'{_t}| - Shutting Down')
    rclpy.shutdown()
    if V: print(f'{_t}| - Done')


# =============================================================================
# Test 04 - Run all 3 Cameras
# =============================================================================
def test04_run_all_cameras(
        verbose: int = -1
) -> None:
    '''
    Test 4 - Run all 3 Cameras
    -
    Connects and runs all 3 `Camera` objects on Baxter.

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
    if V: print(f'{_t}Camera Test 4 - Run all 3 Cameras')

    # rclpy initialize
    if V: print(f'| - Initializing RCLPY')
    rclpy.init()

    # creating cameras
    if V: print(f'{_t}| - Creating Cameras')
    cam_head = Camera(
        Topics.Camera.HEAD,
        verbose = _sub_v
    )
    cam_left = Camera(
        Topics.Camera.LEFT,
        verbose = _sub_v
    )
    cam_right = Camera(
        Topics.Camera.RIGHT,
        verbose = _sub_v
    )
    all_nodes: List[Node] = [
        cam_head,
        cam_left,
        cam_right,
    ]

    # spinning rclpy
    if V: print(f'| - Creating Executors')
    _exec = rclpy.executors.MultiThreadedExecutor()
    for node in all_nodes:
        _exec.add_node(node)
    if V: print(f'| - Spinning')
    _exec.spin()

    # shutting down
    if V: print(f'| - Destroying Nodes')
    for node in all_nodes:
        node.destroy_node()
    if V: print(f'| - Shutting Down')
    rclpy.shutdown()
    if V: print(f'| - Done')


# =============================================================================
# Test 05 - Stream Image Data directly from 3 Cameras
# =============================================================================
def test05_display_image_all(
        verbose: int = -1
) -> None:
    '''
    Test 5 - Display All Camera Data
    -
    Connects with every `Camera` object on Baxter and continuously updates
    OpenCV image output showing what the camera is showing.

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
    if V: print(f'{_t}Camera Test 5 - Display Single Camera Data')

    # create camera data subscriber
    if V: print(f'{_t}| - Creating Camera Subscriber Class')
    class Camera_Subscriber(Node):
        def __init__(
                self,
                side: str
        ) -> None:
            self.side: str = side
            self.node_name: str = self.side.replace('/', '_')
            super().__init__(self.node_name)
            self.subscription = self.create_subscription(
                MSG_Image,
                f'/cameras/{self.side}/image',
                self.listener_callback,
                10
            )

        def listener_callback(self, _msg: MSG_Image) -> None:
            if V: print(f'{_t}| - Camera Data Read')
            # convert message data type
            if V: print(f'{_t}\t| - Converting Message Data to Custom Type')
            msg = MSG_Image.from_msg(_msg)

            # convert to numpy array
            if V: print(f'{_t}\t| - Converting Image to NumPy Array')
            img_arr = numpy.array(msg.data, dtype = numpy.uint8)
            if V: print(f'{_t}\t\t| - Image 1D: Shape = {img_arr.shape}')

            # convert to 3D array
            if V: print(f'{_t}\t| - Converting 1D to 3D')
            img = numpy.reshape(
                img_arr,
                (
                    msg.height,
                    msg.width,
                    msg.channels,
                )
            )
            if V: print(f'{_t}\t\t| - Image 3D: Shape = {img_arr.shape}')

            # convert colour
            if V: print(f'{_t}\t| - Converting Image Colour')
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            if V: 
                print(
                    f'{_t}\t\t| - Image 3D Colour Corrected: Shape = ' \
                        + f'{img_arr.shape}'
                )
            
            # display image
            if V: print(f'{_t}\t| - Displaying Image')
            cv2.imshow(f'Camera Test {self.side} Image', img)

            if (cv2.waitKey(1) & 0xff) == 27:
                raise ConnectionError

    # initialize rclpy
    if V: print(f'{_t}| - Initialize RCLPY')
    rclpy.init()

    # create subscriber
    if V: print(f'{_t}| - Creating Camera Subscriber')
    # sub = Camera_Subscriber()
    sub_l = Camera_Subscriber(Topics.Camera.LEFT)
    sub_r = Camera_Subscriber(Topics.Camera.RIGHT)
    sub_h = Camera_Subscriber(Topics.Camera.HEAD)

    # spin
    if V: print(f'{_t}| - Spinning')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(sub_l)
    executor.add_node(sub_r)
    executor.add_node(sub_h)

    try:
        executor.spin()
    except ConnectionError:
        pass
    # rclpy.spin(sub)

    # end
    # if V: print(f'{_t}| - Destroying Node')
    # sub.destroy_node()
    if V: print(f'{_t}| - Shutting Down')
    rclpy.shutdown()
    if V: print(f'{_t}| - Done')


# =============================================================================
# Test 06 - Stream Image Data directly from 3 Cameras (using Interface)
# =============================================================================
def test06_display_image_all_interface(
        verbose: int = -1
) -> None:
    '''
    Test 6 - Display All Camera Data using Interface
    -
    Connects with every `Camera` object on Baxter and continuously updates
    OpenCV image output showing what the camera is showing.

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
    if V: print(f'{_t}Camera Test 6 - Display Single Camera Data')

    # initialize rclpy
    if V: print(f'{_t}| - Initialize RCLPY')
    rclpy.init()

    # create subscriber
    if V: print(f'{_t}| - Creating Camera Subscriber')
    cam_l = Camera(
        Topics.Camera.LEFT,
        verbose = _sub_v,
        image_display_rt = True
    )
    cam_r = Camera(
        Topics.Camera.RIGHT,
        verbose = _sub_v,
        image_display_rt = True
    )
    cam_h = Camera(
        Topics.Camera.HEAD,
        verbose = _sub_v,
        image_display_rt = True
    )
    cams = [cam_l, cam_r, cam_h]

    # spin
    if V: print(f'{_t}| - Spinning')
    executor = rclpy.executors.SingleThreadedExecutor()
    for c in cams:
        executor.add_node(c)

    try:
        executor.spin()
    except ConnectionAbortedError:
        print(f'{_t}| - Image Connection Aborted')
    # rclpy.spin(sub)

    # end
    if V: print(f'{_t}| - Destroying Nodes + Windows')
    for c in cams:
        c.destroy_node()
    cv2.destroyAllWindows()
    if V: print(f'{_t}| - Shutting Down')
    rclpy.shutdown()
    if V: print(f'{_t}| - Done')


# =============================================================================
# Test 07 - Stream Image Data using Service/Client (using Interface)
# =============================================================================
def test07_service_client_stream(
        verbose: int = -1
) -> None:
    '''
    Test 7 - Stream Image Data using Service/Client (using Interface)
    -
    Connects with every `Camera` object on Baxter and continuously updates
    OpenCV image output showing what the camera is seeing using Service and
    Client Camera Interface Nodes.

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
    if V: print(f'{_t}Camera Test 7 - Stream Images using Service/Clients')

    # initialize rclpy
    if V: print(f'{_t}| - Initialize RCLPY')
    rclpy.init()

    # create cameras
    if V: print(f'{_t}| - Creating Cameras')
    cams = [
        Camera(
            _topic,
            verbose = _sub_v
        )
        for _topic in [
            Topics.Camera.LEFT,
            Topics.Camera.RIGHT,
            # Topics.Camera.HEAD,
        ]
    ]

    # create clients
    if V: print(f'{_t}| - Creating Clients')
    clients = [
        Client_Camera(
            _topic,
            Services.Camera.IMAGE_DATA,
            verbose = _sub_v,
            stream_video = True
        )
        for _topic in [
            Topics.Camera.LEFT,
            Topics.Camera.RIGHT,
            # Topics.Camera.HEAD,
        ]
    ]

    # create overall nodes list
    if V: print(f'{_t}| - Creating Overall Nodes List')
    nodes = cams + clients

    # spin
    if V: print(f'{_t}| - Spinning (Press ESC on an Image Window to STOP)')
    executor = rclpy.executors.MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except ConnectionAbortedError:
        print(f'{_t}| - Image Connection Aborted')

    # end
    if V: print(f'{_t}| - Destroying Nodes + Windows')
    for node in nodes:
        node.destroy_node()
    cv2.destroyAllWindows()
    if V: print(f'{_t}| - Shutting Down')
    rclpy.shutdown()
    if V: print(f'{_t}| - Done')


# =============================================================================
# Main Function
# =============================================================================
def main(args: None = None):
    test_num: int = 1
    v: int = 0

    if len(sys.argv) >= 2:
        test_num = int(sys.argv[1])
    if len(sys.argv) >= 3:
        v = int(sys.argv[2])

    if test_num == 1:
        test01_read_images(v)
    elif test_num == 2:
        test02_display_images(v)
    elif test_num == 3:
        test03_display_image_singular(v)
    elif test_num == 4:
        test04_run_all_cameras(v)
    elif test_num == 5:
        test05_display_image_all(v)
    elif test_num == 6:
        test06_display_image_all_interface(v)
    elif test_num == 7:
        test07_service_client_stream(v)
    else:
        raise ValueError(
            f'test_num (argument 1 value) invalid: test_num={repr(test_num)}'
        )

if __name__ == '__main__':
    sys.exit(main())


# =============================================================================
# End of File
# =============================================================================
