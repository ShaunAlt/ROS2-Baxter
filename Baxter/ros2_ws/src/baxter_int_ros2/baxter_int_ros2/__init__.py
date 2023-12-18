#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
ROS2 Baxter Interface Module
-
Contains object, method, and variable definitions required for interacting with
the Baxter robot using ROS2.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# used for type-hinting
from typing import (
    Any,
    Callable,
    cast,
    Dict,
    List,
    Optional,
    Tuple,
    TYPE_CHECKING,
    Union,
)

# used for numpy arrays
import numpy

# used for image processing
import cv2

# used for arrays
from array import (
    array,
)

# used for logging
from logging import (
    Logger,
)

# used for JSON encoding
from json import JSONEncoder

# used for error tracebacks
import traceback

# used for time
import time

# used for validating methods
from inspect import (
    ismethod,
)

# ros2 python connector
from rclpy import ( # type: ignore
    ok as rclpy_ok,
    spin,
    spin_until_future_complete,
)
from rclpy.node import Node as Node # type: ignore
from rclpy.publisher import Publisher as Publisher # type: ignore
from rclpy.service import Service as Service # type: ignore
from rclpy.client import Client as Client # type: ignore
from rclpy.subscription import Subscription as Subscription # type: ignore
from rclpy.timer import ( # type: ignore
    Rate as Rate,
    Timer as Timer,
)

# baxter messages
from baxter_core_msgs.msg import ( # type: ignore
    AnalogIOState as msgAnalogIOState,
    AnalogIOStates as msgAnalogIOStates,
    AnalogOutputCommand as msgAnalogOutputCommand,
    CameraControl as msgCameraControl,
    CameraSettings as msgCameraSettings,
    CollisionAvoidanceState as msgCollisionAvoidanceState,
    CollisionDetectionState as msgCollisionDetectionState,
    DigitalIOState as msgDigitalIOState,
    DigitalIOStates as msgDigitalIOStates,
    DigitalOutputCommand as msgDigitalOutputCommand,
    EndEffectorCommand as msgEndEffectorCommand,
    EndEffectorProperties as msgEndEffectorProperties,
    EndEffectorState as msgEndEffectorState,
    EndpointState as msgEndpointState,
    EndpointStates as msgEndpointStates,
    HeadPanCommand as msgHeadPanCommand,
    HeadState as msgHeadState,
    JointCommand as msgJointCommand,
    NavigatorState as msgNavigatorState,
    NavigatorStates as msgNavigatorStates,
)
from builtin_interfaces.msg import ( # type: ignore
    Time as msgTime,
)
from geometry_msgs.msg import ( # type: ignore
    Point as msgPoint,
    Pose as msgPose,
    Quaternion as msgQuaternion,
    Twist as msgTwist,
    Vector3 as msgVector3,
    Wrench as msgWrench,
)
from sensor_msgs.msg import ( # type: ignore
    Image as msgImage,
)
from std_msgs.msg import ( # type: ignore
    Header as msgHeader,
)

# custom messages
from baxter_int_ros2_support.msg import ( # type: ignore
    CameraData as msgCameraData,
)

# baxter services
from baxter_core_msgs.srv import ( # type: ignore
    CloseCamera as srvCloseCamera,
    ListCameras as srvListCameras,
    OpenCamera as srvOpenCamera,
)

# custom services
from baxter_int_ros2_support.srv import ( # type: ignore
    CameraData as srvCameraData,
)

# used for timing
import time

# used for weakset and weakdictionaries
from weakref import WeakKeyDictionary
try:
    from weakref import WeakSet
except ImportError:
    from _weakrefset import WeakSet

if TYPE_CHECKING:
    from cv2.typing import (
        MatLike,
    )
else:
    MatLike = Any


# =============================================================================
# Main ROS2 Interface Object
# =============================================================================
class ROS2_obj():
    '''
    Main ROS2 Interface Object
    -
    Main parent ROS2 interface object which contains the base methods and 
    attributes that all ROS2 interface objects contain.

    Attributes
    -
    None

    Methods
    -
    - _get_data(short = False) : `_DATA`
        - Creates a list of data points to be printed out by the `__str__` and
            `__repr__` instance methods.
    - __str__() : `str`
        - Creates a short stringified version of the current ROS2 object.
    - __repr__() : `str`
        - Creates a long, more descriptive version of the `__str__()` output.
    '''

    # ===============
    # Get Object Data
    def _get_data(
            self,
            short: bool = False
    ) -> '_DATA':
        '''
        Get Object Data
        -
        Creates a list of data points to be printed out by the `__str__` and
        `__repr__` instance methods.

        Parameters
        -
        - short : `bool`
            - Whether to produce the shortened list of data used in `__str__`,
                or the longer list used in `__repr__`.

        Returns
        -
        `_DATA`
            - List of key and value pairs of the data in the object.
        '''

        raise NotImplementedError('ROS2 Base Get Data Invalid.')

    # ============
    # Short String
    def __str__(self) -> str:
        '''
        Short String
        -
        Creates a short stringified version of the current ROS2 object.

        Parameters
        -
        None

        Returns
        -
        `str`
            - Single-line string of the current object.
        '''

        # initialize variables
        _key: str
        _val: Any
        data: _DATA = self._get_data(True)
        i: int
        output: str = f'<{self.__class__.__name__}'

        # create output string
        for i, _key in enumerate(data):
            _val = data[_key]
            try:
                if len(_val) > 50:
                    _val = f'{type(_val)} with ({len(_val)}) items.'
            except:
                pass
            if i > 0:
                output += ','
            output += f' {_key} = {_val}'
        output += ' />'

        return output
    
    # ===========
    # Long String
    def __repr__(self) -> str:
        '''
        Long String
        -
        Creates a long, more descriptive version of the `__str__()` output.

        Parameters
        -
        None

        Returns
        -
        `str`
            - Multi-line string of the current object
        '''

        # initialize variables
        _key: str
        _val: Any
        _val_str: str
        data: _DATA = self._get_data(False)
        i: int
        output: str = f'<{self.__class__.__name__}\n'

        # create output string
        for i, _key in enumerate(data):
            _val = data[_key]
            try:
                if len(_val) > 50 and not isinstance(_val, str):
                    _val = f'{type(_val)} with ({len(_val)}) items.'
            except:
                pass
            # indent internal data so it is more collapsible
            _val_str = str(_val).replace('\n', '\n\t\t')
            output += f'\t{_key} = {_val_str}'
            if i != len(data) - 1:
                output += ','
            output += '\n'
        output += '/>'

        return output


# =============================================================================
# Main ROS2 Interface Node Object
# =============================================================================
class ROS2_Node(ROS2_obj, Node):
    '''
    Main ROS2 Interface Node
    -
    Main parent ROS2 interface object to all children Node objects.

    Attributes
    -
    - _node_name : `str`
        - Unique name of the node.
    - _verbose : `int`
        - Defaults to `-1` which means no verbosity. Any value 0 or greater
            represents the number of tabs to indent logs by.
    - _V : `bool`
        - Whether or not to display verbose logs.
    - _verbose_sub : `int`
        - If sub-routines are called, they will be called with this value of
            verbosity which will either keep non-verbose or else indent the
            output by an extra tab as required.
    
    `Node` Methods
    -
    - create_publisher(msg, topic, queue=10) : `Publisher`
        - Creates a publisher which publishes data to the given topic. Return
            value needs to be stored in an instance variable so that commands
            can be published to the topic.
    - create_subscription(msg, topic, callback_func, queue=10) : `Subscriber`
        - Creates a subscriber for the given topic which runs the callback
            function whenever data is pushed onto the topic. Return value
            does not require storing.
    - create_timer(timer_sec, callback_func) : `Timer`
        - Creates a timer which runs the callback function after each interval.
            Return value does not required storing.
    - destroy_node() : `None`
        - Destroys the current node.
    - get_logger() : `Logger`
        - Creates a logger which can be used for logging and debugging
            purposes.

    `ROS2_obj` Methods
    -
    - _get_data(short = False) : `_DATA`
        - Creates a list of data points to be printed out by the `__str__` and
            `__repr__` instance methods.
    - __str__() : `str`
        - Creates a short stringified version of the current ROS2 object.
    - __repr__() : `str`
        - Creates a long, more descriptive version of the `__str__()` output.
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            _node_name: str,
            _verbose: int = -1
    ) -> None:
        self._node_name: str = _node_name.replace('/', '__')
        self._verbose: int = _verbose
        self._V: bool = _verbose >= 0
        self._verbose_sub: int = {True: _verbose + 1, False: -1}[self._V]
        super().__init__(self._node_name)

    # =========
    # Node Name
    @property
    def node_name(self) -> str:
        ''' Node Name. '''
        return self._node_name

    # ================
    # Create Publisher
    def create_pub(
            self,
            msg: '_MSG',
            topic: str,
            queue: int = 10
    ) -> Publisher:
        '''
        Create Publisher
        -
        Creates a publisher which publishes data to the given topic. Return
        value needs to be stored in an instance variable so that commands can
        be published to the topic.

        Parameters
        -
        - msg : `_MSG`
            - Message datatype being published. This MUST be the original
                object and not the custom one from `.msgs` defined in this
                module.
        - topic : `str`
            - String value of the topic being published to.
        - queue : `int`
            - TODO: Unclear what this is actually for, defaults to 10.

        Returns
        -
        `Publisher`
            - ROS2 publisher for the specified topic.
        '''

        return super().create_publisher(
            msg,
            topic,
            queue
        )
    
    # =================
    # Create Subscriber
    def create_sub(
            self,
            msg: '_MSG',
            topic: str,
            callback_func: Callable[['_MSG'], None],
            queue: int = 10
    ) -> Subscription:
        '''
        Create Subscriber
        -
        Creates a subscriber for the given topic which runs the callback
        function whenever data is pusehd onto the topic. Return value does
        not require storing.

        Parameters
        -
        - msg : `_MSG`
            - Message datatype being subscribed to. Can be either the original
                object or the custom one defined in this module in `.msgs`.
        - topic : `str`
            - String value of the topic being subscribed to.
        - callback_func : `Callable[[_MSG], None]`
            - Callback function to run when the subscriber gets data.
        - queue : `int`
            - TODO: Unclear what this is actually for, defaults to 10.

        Returns
        -
        `Subscriber`
            - ROS2 Subscriber for the specified topic.
        '''

        return super().create_subscription(
            msg,
            topic,
            callback_func,
            queue
        )
    
    # ============
    # Create Timer
    def create_tim(
            self,
            timer_sec: float,
            callback_func: Callable[[], None]
    ) -> Timer:
        '''
        Create Timer
        -
        Creates a timer which runs the callback function after each interval.
        Return value does not required storing.

        Parameters
        -
        - timer_sec : `float`
            - Decimal number of seconds to wait between each callback functio
                call.
        - callback_func : `Callable[[], None]`
            - Callback function to run each time the timer is activated.

        Returns
        -
        `Timer`
            - ROS2 Timer.
        '''

        return super().create_timer(
            timer_sec,
            callback_func
        )

    # ============
    # Destroy Node
    def destroy_n(self) -> None:
        '''
        Destroy Node
        -
        Destroys the current node.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        super().destroy_node()

    # ==========
    # Get Logger
    def get_l(self) -> Logger:
        '''
        Get Logger
        -
        Creates a logger which can be used for logging and debugging purposes.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        return cast(
            Logger,
            super().get_logger()
        )


# =============================================================================
# Definitions
# =============================================================================
_DATA = Dict[str, Any]
_MSG = Union[
    # - baxter_core_msgs
    msgAnalogIOState,
    msgAnalogIOStates,
    msgAnalogOutputCommand,
    msgCameraControl,
    msgCameraSettings,
    msgCollisionAvoidanceState,
    msgCollisionDetectionState,
    msgDigitalIOState,
    msgDigitalIOStates,
    msgDigitalOutputCommand,
    msgEndEffectorCommand,
    msgEndEffectorProperties,
    msgEndEffectorState,
    msgEndpointState,
    msgEndpointStates,
    msgHeadPanCommand,
    msgHeadState,
    msgJointCommand,
    msgNavigatorState,
    msgNavigatorStates,

    # - builtin_interfaces
    msgTime,

    # - geometry_msgs
    msgPoint,
    msgPose,
    msgQuaternion,
    msgTwist,
    msgVector3,
    msgWrench,

    # - sensor_msgs
    msgImage,

    # - std_msgs
    msgHeader,
]
_TIMER = Tuple[float, Callable[[Node], None]]
class Settings():
    CHECK_VERSION = True
    HEAD_PAN_ANGLE_TOLERANCE: float = 0.1396263401
    JOINT_ANGLE_TOLERANCE: float = 0.008726646
    SDK_VERSION = '1.2.0'
    # Version Compatibility Maps - {current: compatible}
    VERSIONS_SDK2ROBOT = {'1.2.0': ['1.2.0']}
    VERSIONS_SDK2GRIPPER = {
        '1.2.0': {
            'warn': '2014/5/20 00:00:00',  # Version 1.0.0
            'fail': '2013/10/15 00:00:00', # Version 0.6.2
        },
    }
class Topics():
    ''' Collection of all ROS2 Baxter Topics. '''
    class Camera():
        ''' Camera Topics. '''
        HEAD: str = 'head_camera'
        LEFT: str = 'left_hand_camera'
        RIGHT: str = 'right_hand_camera'
        IMAGE_DATA: str = 'image_data'
        IMAGE_DATA_HEAD: str = f'{HEAD}/{IMAGE_DATA}'
        IMAGE_DATA_LEFT: str = f'{LEFT}/{IMAGE_DATA}'
        IMAGE_DATA_RIGHT: str = f'{RIGHT}/{IMAGE_DATA}'
        PROCESS_DATA_TABLE: str = 'processed_table'
        PROCESS_DATA_TABLE_HEAD: str = f'{HEAD}/{PROCESS_DATA_TABLE}'
        PROCESS_DATA_TABLE_LEFT: str = f'{LEFT}/{PROCESS_DATA_TABLE}'
        PROCESS_DATA_TABLE_RIGHT: str = f'{RIGHT}/{PROCESS_DATA_TABLE}'
        ALL: List[str] = [
            HEAD,
            LEFT,
            RIGHT,
        ]
        ALL_IMAGE_VIEWS: List[str] = [
            IMAGE_DATA_HEAD,
            IMAGE_DATA_LEFT,
            IMAGE_DATA_RIGHT,
            PROCESS_DATA_TABLE_HEAD,
            PROCESS_DATA_TABLE_LEFT,
            PROCESS_DATA_TABLE_RIGHT,
        ]

    class DigitalIO():
        ''' Digital IO Topics. '''
        HEAD_GREEN_LIGHT: str = 'head_green_light'
        HEAD_LCD_AUTO_CONFIG: str = 'head_lcd_auto_config'
        HEAD_RED_LIGHT: str = 'head_red_light'
        HEAD_YELLOW_LIGHT: str = 'head_yellow_light'
        LEFT_BLOW: str = 'left_blow'
        LEFT_BUTTON_BACK: str = 'left_button_back'
        LEFT_BUTTON_OK: str = 'left_button_ok'
        LEFT_BUTTON_SHOW: str = 'left_button_show'
        LEFT_HAND_CAMERA_POWER: str = 'left_hand_camera_power'
        LEFT_INNER_LIGHT: str = 'left_inner_light'
        LEFT_LOWER_BUTTON: str = 'left_lower_button'
        LEFT_LOWER_CUFF: str = 'left_lower_cuff'
        LEFT_OUTER_LIGHT: str = 'left_outer_light'
        LEFT_PNEUMATIC: str = 'left_pneumatic'
        LEFT_SHOULDER_BUTTON: str = 'left_shoulder_button'
        LEFT_SUCK: str = 'left_suck'
        LEFT_UPPER_BUTTON: str = 'left_upper_button'
        LIMIT_SWITCH_1: str = 'limit_switch_1'
        LIMIT_SWITCH_2: str = 'limit_switch_2'
        LIMIT_SWITCH_3: str = 'limit_switch_3'
        MOTOR_FAULT_SIGNAL: str = 'motor_fault_signal'
        RIGHT_BLOW: str = 'right_blow'
        RIGHT_BLUE_LIGHT: str = 'right_blue_light'
        RIGHT_BUTTON_BACK: str = 'right_button_back'
        RIGHT_BUTTON_OK: str = 'right_button_ok'
        RIGHT_BUTTON_SHOW: str = 'right_button_show'
        RIGHT_HAND_CAMERA_POWER: str = 'right_hand_camera_power'
        RIGHT_INNER_LIGHT: str = 'right_inner_light'
        RIGHT_LOWER_BUTTON: str = 'right_lower_button'
        RIGHT_LOWER_CUFF: str = 'right_lower_cuff'
        RIGHT_OUTER_LIGHT: str = 'right_outer_light'
        RIGHT_PNEUMATIC: str = 'right_pneumatic'
        RIGHT_SHOULDER_BUTTON: str = 'right_shoulder_button'
        RIGHT_SUCK: str = 'right_suck'
        RIGHT_UPPER_BUTTON: str = 'right_upper_button'
        TORSO_BRAKE: str = 'torso_brake'
        TORSO_BRAKE_SENSOR: str = 'torso_brake_sensor'
        TORSO_CAMERA_POWER: str = 'torso_camera_power'
        TORSO_DIGITAL_INPUT0: str = 'torso_digital_input0'
        TORSO_FOOT_PEDAL: str = 'torso_foot_pedal'
        TORSO_LEFT_BUTTON_BACK: str = 'torso_left_button_back'
        TORSO_LEFT_BUTTON_OK: str = 'torso_left_button_ok'
        TORSO_LEFT_BUTTON_SHOW: str = 'torso_left_button_show'
        TORSO_LEFT_INNER_LIGHT: str = 'torso_left_inner_light'
        TORSO_LEFT_OUTER_LIGHT: str = 'torso_left_outer_light'
        TORSO_PROCESS_SENSE0: str = 'torso_process_sense0'
        TORSO_PROCESS_SENSE1: str = 'torso_process_sense1'
        TORSO_RIGHT_BUTTON_BACK: str = 'torso_right_button_back'
        TORSO_RIGHT_BUTTON_OK: str = 'torso_right_button_ok'
        TORSO_RIGHT_BUTTON_SHOW: str = 'torso_right_button_show'
        TORSO_RIGHT_INNER_LIGHT: str = 'torso_right_inner_light'
        TORSO_RIGHT_OUTER_LIGHT: str = 'torso_right_outer_light'
        TORSO_SAFETY_STOP: str = 'torso_safety_stop'
        TORSO_UI_OUTPUT0: str = 'torso_ui_output0'
        TORSO_UI_OUTPUT1: str = 'torso_ui_output1'
        TORSO_UI_OUTPUT2: str = 'torso_ui_output2'
        TORSO_UI_OUTPUT3: str = 'torso_ui_output3'
        ALL: List[str] = [
            HEAD_GREEN_LIGHT,
            HEAD_LCD_AUTO_CONFIG,
            HEAD_RED_LIGHT,
            HEAD_YELLOW_LIGHT,
            LEFT_BLOW,
            LEFT_BUTTON_BACK,
            LEFT_BUTTON_OK,
            LEFT_BUTTON_SHOW,
            LEFT_HAND_CAMERA_POWER,
            LEFT_INNER_LIGHT,
            LEFT_LOWER_BUTTON,
            LEFT_LOWER_CUFF,
            LEFT_OUTER_LIGHT,
            LEFT_PNEUMATIC,
            LEFT_SHOULDER_BUTTON,
            LEFT_SUCK,
            LEFT_UPPER_BUTTON,
            LIMIT_SWITCH_1,
            LIMIT_SWITCH_2,
            LIMIT_SWITCH_3,
            MOTOR_FAULT_SIGNAL,
            RIGHT_BLOW,
            RIGHT_BLUE_LIGHT,
            RIGHT_BUTTON_BACK,
            RIGHT_BUTTON_OK,
            RIGHT_BUTTON_SHOW,
            RIGHT_HAND_CAMERA_POWER,
            RIGHT_INNER_LIGHT,
            RIGHT_LOWER_BUTTON,
            RIGHT_LOWER_CUFF,
            RIGHT_OUTER_LIGHT,
            RIGHT_PNEUMATIC,
            RIGHT_SHOULDER_BUTTON,
            RIGHT_SUCK,
            RIGHT_UPPER_BUTTON,
            TORSO_BRAKE,
            TORSO_BRAKE_SENSOR,
            TORSO_CAMERA_POWER,
            TORSO_DIGITAL_INPUT0,
            TORSO_FOOT_PEDAL,
            TORSO_LEFT_BUTTON_BACK,
            TORSO_LEFT_BUTTON_OK,
            TORSO_LEFT_BUTTON_SHOW,
            TORSO_LEFT_INNER_LIGHT,
            TORSO_LEFT_OUTER_LIGHT,
            TORSO_PROCESS_SENSE0,
            TORSO_PROCESS_SENSE1,
            TORSO_RIGHT_BUTTON_BACK,
            TORSO_RIGHT_BUTTON_OK,
            TORSO_RIGHT_BUTTON_SHOW,
            TORSO_RIGHT_INNER_LIGHT,
            TORSO_RIGHT_OUTER_LIGHT,
            TORSO_SAFETY_STOP,
            TORSO_UI_OUTPUT0,
            TORSO_UI_OUTPUT1,
            TORSO_UI_OUTPUT2,
            TORSO_UI_OUTPUT3,
        ]

    class Gripper():
        ''' Gripper Topics. '''
        LEFT: str = 'left'
        RIGHT: str = 'right'
        ALL: List[str] = [
            LEFT,
            RIGHT,
        ]

        class SubTopics():
            ''' Gripper Sub-Topics. '''
            GET_PROPS: str = 'properties'
            GET_STATE: str = 'state'
            PREFIX: str = 'robot/end_effector'
            SET_CMD: str = 'command'
            SET_PROPS: str = 'rsdk/set_properties'
            SET_STATE: str = 'rsdk/set_state'
            SUFFIX: str = '_gripper'

# =============================================================================
# Sub-Module Imports
# =============================================================================

from .dataflow import (
    df_wait,
    Signal,
)
from .msgs import (
    # baxter messages
    AnalogIOState as MSG_AnalogIOState,
    AnalogIOStates as MSG_AnalogIOStates,
    AnalogOutputCommand as MSG_AnalogOutputCommand,
    CameraControl as MSG_CameraControl,
    CameraSettings as MSG_CameraSettings,
    CollisionAvoidanceState as MSG_CollisionAvoidanceState,
    CollisionDetectionState as MSG_CollisionDetectionState,
    DigitalIOState as MSG_DigitalIOState,
    DigitalIOStates as MSG_DigitalIOStates,
    DigitalOutputCommand as MSG_DigitalOutputCommand,
    EndEffectorCommand as MSG_EndEffectorCommand,
    EndEffectorProperties as MSG_EndEffectorProperties,
    EndEffectorState as MSG_EndEffectorState,
    EndpointState as MSG_EndpointState,
    EndpointStates as MSG_EndpointStates,
    HeadPanCommand as MSG_HeadPanCommand,
    HeadState as MSG_HeadState,
    Image as MSG_Image,
    JointCommand as MSG_JointCommand,
    NavigatorState as MSG_NavigatorState,
    NavigatorStates as MSG_NavigatorStates,
    # ROS2 messages
    Header as MSG_Header,
    Point as MSG_Point,
    Pose as MSG_Pose,
    Quaternion as MSG_Quaternion,
    Time as MSG_Time,
    Twist as MSG_Twist,
    Vector3 as MSG_Vector3,
    Wrench as MSG_Wrench,
    # Custom messages
    CameraData as MSG_CameraData,
)
from .srvs import (
    # baxter_core_msgs
    CloseCamera as SRV_CloseCamera,
    ListCameras as SRV_ListCameras,
    OpenCamera as SRV_OpenCamera,
    # baxter_int_ros2
    CameraData as SRV_CameraData,
)
from .camera import (
    Camera,
    Image_Processor,
    Image_Viewer,
)
from .digital_io import (
    DigitalIO,
)
from .gripper import (
    Gripper,
)

# =============================================================================
# End of File
# =============================================================================
