#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
ROS2 Baxter Message Type Definitions
-
Contains message type definitions used for type-hinting in ROS2.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

from . import (
    # - typing
    _DATA,
    Any,
    cast,
    List,
    Tuple,

    # - array
    array,

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

    # - baxter_int_ros2_support
    msgCameraData,

    # - .
    ROS2_obj,
    _MSG,
)


# =============================================================================
# ROS2 Messages - Generic Parent Class
# =============================================================================
class ROS2_msg(ROS2_obj):
    '''
    ROS2 Message Parent
    -
    Main ROS2 Message parent which contains the base methods and attributes
    used by all of the ROS2_msg objects in the same module.

    Attributes
    -
    - ALL_TYPES : `list[str]`
        - List of all valid types that messages can be.

    Methods
    -
    - from_msg(msg) : `ROS2_msg`
        - Static Method.
        - Creates an instance of the message from the ROS2 message instead of
            the `__init__` method.
    - _validate_input(data, _type, options=[]) : `str`
        - Validates a particular data input and makes sure it is the correct
            type.
    '''

    # =========
    # Constants
    TYPE_ANALOGIOSTATE: str = 'baxter_core_msgs.AnalogIOState'
    TYPE_ARRAY: str = 'array'
    TYPE_BOOL: str = 'bool'
    TYPE_CAMERACONTROL: str = 'baxter_core_msgs.CameraControl'
    TYPE_FLOAT32: str = 'float32'
    TYPE_FLOAT64: str = 'float64'
    TYPE_HEADER: str = 'std_msgs.Header'
    TYPE_INT32: str = 'int32'
    TYPE_INT8: str = 'int8'
    TYPE_LIST: str = 'list'
    TYPE_OTHER: str = 'other type - dont check'
    TYPE_POINT: str = 'geometry_msgs.Point'
    TYPE_POSE: str = 'geometry_msgs.Pose'
    TYPE_QUARTERNION: str = 'geometry_msgs.Quaternion'
    TYPE_STRING: str = 'string'
    TYPE_TIME: str = 'builtin_interfaces.Time'
    TYPE_TWIST: str = 'geometry_msgs.Twist'
    TYPE_UINT8: str = 'uint8'
    TYPE_UINT16: str = 'uint16'
    TYPE_UINT32: str = 'uint32'
    TYPE_VECTOR3: str = 'geometry_msgs.Vector3'
    TYPE_WRENCH: str = 'geometry_msgs.Wrench'
    ALL_TYPES: List[str] = [
        TYPE_ANALOGIOSTATE,
        TYPE_ARRAY,
        TYPE_BOOL,
        TYPE_CAMERACONTROL,
        TYPE_FLOAT32,
        TYPE_FLOAT64,
        TYPE_HEADER,
        TYPE_INT32,
        TYPE_INT8,
        TYPE_LIST,
        TYPE_OTHER,
        TYPE_POINT,
        TYPE_POSE,
        TYPE_QUARTERNION,
        TYPE_STRING,
        TYPE_TIME,
        TYPE_TWIST,
        TYPE_UINT8,
        TYPE_UINT16,
        TYPE_UINT32,
        TYPE_VECTOR3,
        TYPE_WRENCH,
    ]
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: _MSG) -> 'ROS2_msg':
        '''
        Create from MSG
        -
        Static Method. Creates an instance of the message from the ROS2 message
        instead of the `__init__` method.

        Parameters
        -
        msg : `_MSG`
            ROS2 message used for creating the message object.
        
        Returns
        -
        `cls`
            - New instance of the current message class.
        '''

        raise NotImplementedError('ROS2 Base Message - Get from MSG Invalid.')

    # ==============
    # Create Message
    def create_msg(self) -> _MSG:
        '''
        Create Message
        -
        Creates a ROS2 topic message from the current message data in the
        current object instance.

        Parameters
        -
        None

        Returns
        -
        `_MSG`
            - New ROS2 message of the correct datatype.
        '''

        raise NotImplementedError('Base ROS2 Message create_msg invalid.')

    # ==============
    # Validate Input
    def _validate_input(
            self,
            data: Any,
            _type: str,
            nullable: bool = False,
            options: List[Any] = []
    ) -> str:
        '''
        Validate Input
        -
        Validates a particular data input and makes sure it is the correct
        type.

        Parameters
        -
        - data : `Any`
            - Value of the data being validated.
        - _type : `str`
            - String representation of the type of data being validated.
        - nullable : `bool`
            - Whether or not the data is allowed to be `None`. Defaults to
                `False`.
        - options : `list[Any]`
            - Defaults to an empty list, which means no set list of values that
                must be selected from. If list isn't empty, then the data value
                must be one of the options in the list.

        Returns
        -
        `str`
            - If the data is valid, will return an empty string. Otherwise,
                will return a string containing the exact problem with the data
                value.
        '''

        # validate _type
        if _type not in self.ALL_TYPES:
            return (
                f'Invalid type specification: _type = {_type} not in ALL_TYPES'
            )
        
        # validate if data is NULL (if required)
        if (not nullable) and (data is None):
            return (
                'Invalid data: data is None'
            )
        
        # validate is data is in options (if required)
        if (len(options) > 0) and (data not in options):
            return (
                f'Invalid data: data = {data} not in options = {options}'
            )

        # validate datatype
        if (
                ( # AnalogIOState
                    (_type == self.TYPE_ANALOGIOSTATE)
                    and (not isinstance(data, AnalogIOState))
                )
                or ( # array
                    (_type == self.TYPE_ARRAY)
                    and (not isinstance(data, array))
                )
                or ( # bool
                    (_type == self.TYPE_BOOL)
                    and (not isinstance(data, bool))
                )
                or ( # CameraControl
                    (_type == self.TYPE_CAMERACONTROL)
                    and (not isinstance(data, CameraControl))
                )
                or ( # float32
                    (_type == self.TYPE_FLOAT32)
                    and (
                        (not (
                            isinstance(data, float)
                            or isinstance(data, int)
                        ))
                        or (data < -3.4e38)
                        or (data > 3.4e38)
                    )
                )
                or ( # float64
                    (_type == self.TYPE_FLOAT64)
                    and (
                        (
                            not (
                                isinstance(data, float)
                                or isinstance(data, int)
                            )
                            or (data < -1.7e308)
                            or (data > 1.7e308)
                        )
                    )
                )
                or ( # Header
                    (_type == self.TYPE_HEADER)
                    and (not isinstance(data, Header))
                )
                or ( # int32
                    (_type == self.TYPE_INT32)
                    and (
                        (not isinstance(data, int))
                        or (data < -2147483648)
                        or (data > 2147483647)
                    )
                )
                or ( # int8
                    (_type == self.TYPE_INT8)
                    and (
                        (not isinstance(data, int))
                        or (data < -128)
                        or (data > 127)
                    )
                )
                or ( # list
                    (_type == self.TYPE_LIST)
                    and (
                        (not isinstance(data, list))
                        or (len(data) == 0)
                    )
                )
                or ( # Point
                    (_type == self.TYPE_POINT)
                    and (not isinstance(data, Point))
                )
                or (
                    (_type == self.TYPE_POSE)
                    and (not isinstance(data, Pose))
                )
                or ( # Quarternion
                    (_type == self.TYPE_QUARTERNION)
                    and (not isinstance(data, Quaternion))
                )
                or ( # string
                    (_type == self.TYPE_STRING)
                    and (
                        (not isinstance(data, str))
                        or (len(data) == 0)
                    )
                )
                or ( # Time
                    (_type == self.TYPE_TIME)
                    and (not isinstance(data, Time))
                )
                or ( # Twist
                    (_type == self.TYPE_TWIST)
                    and (not isinstance(data, Twist))
                )
                or ( # uint8
                    (_type == self.TYPE_UINT8)
                    and (
                        (not isinstance(data, int))
                        or (data < 0)
                        or (data > 255)
                    )
                )
                or ( # uint16
                    (_type == self.TYPE_UINT16)
                    and (
                        (not isinstance(data, int))
                        or (data < 0)
                        or (data > 65535)
                    )
                )
                or ( # uint32
                    (_type == self.TYPE_UINT32)
                    and (
                        (not isinstance(data, int))
                        or (data < 0)
                        or (data > 4294967295)
                    )
                )
                or ( # Vector3
                    (_type == self.TYPE_VECTOR3)
                    and (not isinstance(data, Vector3))
                )
                or ( # Wrench
                    (_type == self.TYPE_WRENCH)
                    and (not isinstance(data, Wrench))    
                )
        ):
            return (
                f'Invalid data (Required {_type}): data = ' \
                    + f'{data} type = {type(data)}'
            )

        return ''


# =============================================================================
# ROS2 Messages
# =============================================================================

# ======
# Header
class Header(ROS2_msg, msgHeader):
    '''
    std_msgs - Header
    -
    Generally used for communicating timestamped data.

    Data
    -
    - stamp : `builtin_interfaces.Time`
    - frame_id : `string`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            stamp: 'Time',
            frame_id: str
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        stamp,
                        self.TYPE_TIME
                    ),
                    self._validate_input(
                        frame_id,
                        self.TYPE_STRING
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG Header Construct: {_str}'
                )

        # construct instance
        super().__init__()
        self.stamp = stamp
        self.frame_id = frame_id

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return cast(
            _DATA,
            {
                True: {
                    'frame_id': self.frame_id,
                    'stamp': (
                        self.stamp.sec,
                        self.stamp.nanosec,
                    ),
                },
                False: {
                    'frame_id': self.frame_id,
                    'stamp': self.stamp,
                }
            }[short]
        )
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgHeader) -> 'Header':
        return Header(
            stamp = Time.from_msg(msg.stamp),
            frame_id = msg.frame_id
        )

# =====
# Point
class Point(ROS2_msg, msgPoint):
    '''
    geometry_msgs - Point
    -
    Contains the position of a point in free space.

    Data
    -
    - x : `float64`
    - y : `float64`
    - z : `float64`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            x: float,
            y: float,
            z: float
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        x,
                        self.TYPE_FLOAT64
                    ),
                    self._validate_input(
                        y,
                        self.TYPE_FLOAT64
                    ),
                    self._validate_input(
                        z,
                        self.TYPE_FLOAT64
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG Point Construct: {_str}'
                )
        
        # construct instance
        super().__init__()
        self.x = x
        self.y = y
        self.z = z

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
        }
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgPoint) -> 'Point':
        return Point(
            x = msg.x,
            y = msg.y,
            z = msg.z
        )

# ====
# Pose
class Pose(ROS2_msg, msgPose):
    '''
    geometry_msgs - Pose
    -
    A representation of a pose in free space, composed of position and
    orientation.

    Data
    -
    - position : `Point`
    - orientation : `Quaternion`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            position: 'Point',
            orientation: 'Quaternion',
            skip_validation: bool = False
    ) -> None:
        # validate data
        if not skip_validation:
            for _str in (
                    [
                        self._validate_input(
                            position,
                            self.TYPE_POINT
                        ),
                        self._validate_input(
                            orientation,
                            self.TYPE_QUARTERNION
                        )
                    ]
            ):
                if _str != '':
                    raise ValueError(
                        f'ROS2-MSG Pose Construct: {_str}'
                    )
            
        # construct instance
        super().__init__()
        self.position = position
        self.orientation = orientation

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return cast(
            _DATA,
            {
                True: {
                    'position': (
                        self.position.x, 
                        self.position.y, 
                        self.position.z,
                    ),
                    'orientation': (
                        self.orientation.x, 
                        self.orientation.y,
                        self.orientation.z,
                        self.orientation.w,
                    ),
                },
                False: {
                    'position': self.position,
                    'orientation': self.orientation,
                }
            }[short]
        )
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgPose) -> 'Pose':
        return Pose(
            position = Point.from_msg(msg.position),
            orientation = Quaternion.from_msg(msg.orientation),
            skip_validation = True
        )

# ==========
# Quaternion
class Quaternion(ROS2_msg, msgQuaternion):
    '''
    geometry_msgs - Quaternion
    -
    Represents an orientation in free space in quarternion form.

    Data
    -
    - x : `float64`
    - y : `float64`
    - z : `float64`
    - w : `float64`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            x: float,
            y: float,
            z: float,
            w: float
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        x,
                        self.TYPE_FLOAT64
                    ),
                    self._validate_input(
                        y,
                        self.TYPE_FLOAT64
                    ),
                    self._validate_input(
                        z,
                        self.TYPE_FLOAT64
                    ),
                    self._validate_input(
                        w,
                        self.TYPE_FLOAT64
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG Quarternion Construct: {_str}'
                )
            
        # construct instance
        super().__init__()
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'w': self.w,
        }
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgQuaternion) -> 'Quaternion':
        return Quaternion(
            x = msg.x,
            y = msg.y,
            z = msg.z,
            w = msg.w
        )

# ====
# Time
class Time(ROS2_msg, msgTime):
    ''' 
    builtin_interfaces - Time 
    -
    Communicates the ROS2 time as defined in the below link:
        https://design.ros2.org/articles/clock_and_time.html

    Data
    -
    - sec : `int32`
    - nanosec : `uint32`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            sec: int,
            nanosec: int
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        sec,
                        self.TYPE_INT32
                    ),
                    self._validate_input(
                        nanosec,
                        self.TYPE_UINT32
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG Time Construct: {_str}'
                )
            
        # construct instance
        super().__init__()
        self.sec = sec
        self.nanosec = nanosec

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return {
            'sec': self.sec,
            'nanosec': self.nanosec,
        }
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgTime) -> 'Time':
        return Time(
            sec = msg.sec,
            nanosec = msg.nanosec
        )

# =====
# Twist
class Twist(ROS2_msg, msgTwist):
    '''
    geometry_msgs - Twist
    -
    Expresses velocity in free space broken into its linear and angular parts.

    Data
    -
    - linear : `Vector3`
    - angular : `Vector3`
    '''
    def __init__(
            self,
            linear: 'Vector3',
            angular: 'Vector3'
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        linear,
                        self.TYPE_VECTOR3
                    ),
                    self._validate_input(
                        angular,
                        self.TYPE_VECTOR3
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG Twist Construct: {_str}'
                )
            
        # construct instance
        super().__init__()
        self.linear = linear
        self.angular = angular

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return cast(
            _DATA,
            {
                True: {
                    'linear': (
                        self.linear.x,
                        self.linear.y,
                        self.linear.z,
                    ),
                    'angular': (
                        self.angular.x,
                        self.angular.y,
                        self.angular.z,
                    ),
                },
                False: {
                    'linear': self.linear,
                    'angular': self.angular,
                },
            }[short]
        )
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgTwist) -> 'Twist':
        return Twist(
            linear = Vector3.from_msg(msg.linear),
            angular = Vector3.from_msg(msg.angular)
        )

# =======
# Vector3
class Vector3(ROS2_msg, msgVector3):
    '''
    geometry_msgs - Vector3
    -
    Represents a vector in free space centred at the origin.

    Data
    -
    - x : `float64`
    - y : `float64`
    - z : `float64`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            x: float,
            y: float,
            z: float
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        x,
                        self.TYPE_FLOAT64
                    ),
                    self._validate_input(
                        y,
                        self.TYPE_FLOAT64
                    ),
                    self._validate_input(
                        z,
                        self.TYPE_FLOAT64
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG Vector3 Construct: {_str}'
                )
            
        # construct instance
        super().__init__()
        self.x = x
        self.y = y
        self.z = z

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
        }
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgVector3) -> 'Vector3':
        return Vector3(
            x = msg.x,
            y = msg.y,
            z = msg.z
        )

# ======
# Wrench
class Wrench(ROS2_msg, msgWrench):
    '''
    geometry_msgs - Wrench
    -
    Represents a force in free space, separated into its linear and angular
    parts.

    Data
    -
    - force : `Vector3`
    - torque : `Vector3`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            force: Vector3,
            torque: Vector3
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        force,
                        self.TYPE_VECTOR3
                    ),
                    self._validate_input(
                        torque,
                        self.TYPE_VECTOR3
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG Wrench Construct: {_str}'
                )
            
        # construct instance
        super().__init__()
        self.force = force
        self.torque = torque

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return cast(
            _DATA,
            {
                True: {
                    'force': (
                        self.force.x,
                        self.force.y,
                        self.force.z,
                    ),
                    'torque': (
                        self.torque.x,
                        self.torque.y,
                        self.torque.z,
                    ),
                },
                False: {
                    'force': self.force,
                    'torque': self.torque,
                },
            }[short]
        )
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgWrench) -> 'Wrench':
        return Wrench(
            force = Vector3.from_msg(msg.force),
            torque = Vector3.from_msg(msg.torque)
        )


# =============================================================================
# Baxter Messages
# =============================================================================

# =============
# AnalogIOState
class AnalogIOState(ROS2_msg, msgAnalogIOState):
    '''
    baxter_core_msgs - AnalogIOState
    -
    Used for reading the state of an AnalogIO on Baxter.

    Data
    -
    - timestamp : `Time`
    - value : `float64`
    - is_input_only : `bool`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            timestamp: 'Time',
            value: float,
            is_input_only: bool
    ):
        # validate data
        for _str in (
                [
                    self._validate_input(
                        timestamp,
                        self.TYPE_TIME
                    ),
                    self._validate_input(
                        value,
                        self.TYPE_FLOAT64
                    ),
                    self._validate_input(
                        is_input_only,
                        self.TYPE_BOOL
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG AnalogIOState Construct: {_str}'
                )
            
        # construct instance
        super().__init__()
        self.timestamp = timestamp
        self.value = value
        self.is_input_only = is_input_only

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return {
            True: {
                'timestamp': (self.timestamp.sec, self.timestamp.nanosec),
                'value': self.value,
                'is_input_only': self.is_input_only,
            },
            False: {
                'timestamp': self.timestamp,
                'value': self.value,
                'is_input_only': self.is_input_only,
            },
        }[short]
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgAnalogIOState) -> 'AnalogIOState':
        return AnalogIOState(
            timestamp = Time.from_msg(msg.timestamp),
            value = msg.value,
            is_input_only = msg.is_input_only
        )

# ==============
# AnalogIOStates
class AnalogIOStates(ROS2_msg, msgAnalogIOStates):
    '''
    baxter_core_msgs - AnalogIOStates
    -
    List of AnalogIOState objects and their names.

    Data
    -
    - names : `list[str]`
    - states : `list[AnalogIOState]`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            names: List[str],
            states: List[AnalogIOState]
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        names,
                        self.TYPE_LIST
                    ),
                    self._validate_input(
                        states,
                        self.TYPE_LIST
                    )
                ] + [
                    self._validate_input(
                        name,
                        self.TYPE_STRING
                    )
                    for name in names
                ] + [
                    self._validate_input(
                        state,
                        self.TYPE_ANALOGIOSTATE
                    )
                    for state in states
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG AnalogIOStates Construct: {_str}'
                )
            
        # construct instance
        super().__init__()
        self.names = names
        self.states = states

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return {
            True: {
                'names': self.names,
                'states': [state.value for state in self.states],
            },
            False: {
                'names': self.names,
                'states': self.states,
            },
        }[short]
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgAnalogIOStates) -> 'AnalogIOStates':
        return AnalogIOStates(
            names = msg.names,
            states = [
                AnalogIOState.from_msg(state)
                for state in msg.states
            ]
        )

# ===================
# AnalogOutputCommand
class AnalogOutputCommand(ROS2_msg, msgAnalogOutputCommand):
    '''
    baxter_core_msgs - AnalogOutputCommand
    -
    Used for writing a command to an AnalogIO on Baxter.

    Data
    -
    - name : `string`
    - value : `uint16`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            name: str,
            value: int
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        name,
                        self.TYPE_STRING
                    ),
                    self._validate_input(
                        value,
                        self.TYPE_UINT16
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG AnalogOutputCommand Construct: {_str}'
                )
            
        # construct instance
        super().__init__()
        self.name = name
        self.value = value

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return {
            'name': self.name,
            'value': self.value
        }
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgAnalogOutputCommand) -> 'AnalogOutputCommand':
        return AnalogOutputCommand(
            name = msg.name,
            value = msg.value
        )

# =============
# CameraControl
class CameraControl(ROS2_msg, msgCameraControl):
    '''
    baxter_core_msgs - Camera Control
    -
    Used for controlling camera settings on Baxter.

    Constants
    -
    - CAMERA_CONTROL_EXPOSURE : `int32`
    - CAMERA_CONTROL_GAIN : `int32`
    - CAMERA_CONTROL_WHITE_BALANCE_R : `int32`
    - CAMERA_CONTROL_WHITE_BALANCE_G : `int32`
    - CAMERA_CONTROL_WHITE_BALANCE_B : `int32`
    - CAMERA_CONTROL_WINDOW_X : `int32`
    - CAMERA_CONTROL_WINDOW_Y : `int32`
    - CAMERA_CONTROL_FLIP : `int32`
    - CAMERA_CONTROL_MIRROR : `int32`
    - CAMERA_CONTROL_RESOLUTION_HALF : `int32`

    Data
    -
    - id : `int32`
    - value : `int32`
    '''

    # =========
    # Constants
    CAMERA_CONTROL_EXPOSURE: int = 100
    CAMERA_CONTROL_GAIN: int = 101
    CAMERA_CONTROL_WHITE_BALANCE_R: int = 102
    CAMERA_CONTROL_WHITE_BALANCE_G: int = 103
    CAMERA_CONTROL_WHITE_BALANCE_B: int = 104
    CAMERA_CONTROL_WINDOW_X: int = 105
    CAMERA_CONTROL_WINDOW_Y: int = 106
    CAMERA_CONTROL_FLIP: int = 107
    CAMERA_CONTROL_MIRROR: int = 108
    CAMERA_CONTROL_RESOLUTION_HALF: int = 109
    def __init__(
            self,
            id: int,
            value: int
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        id,
                        self.TYPE_INT32,
                        options = [
                            self.CAMERA_CONTROL_EXPOSURE,
                            self.CAMERA_CONTROL_GAIN,
                            self.CAMERA_CONTROL_WHITE_BALANCE_R,
                            self.CAMERA_CONTROL_WHITE_BALANCE_G,
                            self.CAMERA_CONTROL_WHITE_BALANCE_B,
                            self.CAMERA_CONTROL_WINDOW_X,
                            self.CAMERA_CONTROL_WINDOW_Y,
                            self.CAMERA_CONTROL_FLIP,
                            self.CAMERA_CONTROL_MIRROR,
                            self.CAMERA_CONTROL_RESOLUTION_HALF,
                        ]
                    ),
                    self._validate_input(
                        value,
                        self.TYPE_INT32
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG CameraControl Construct: {_str}'
                )
            
        # construct instance
        super().__init__()
        self.id = id
        self.value = value

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return {
            'id': self.id,
            'value': self.value
        }

    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgCameraControl) -> 'CameraControl':
        return CameraControl(
            id = msg.id,
            value = msg.value
        )

# ==============
# CameraSettings
class CameraSettings(ROS2_msg, msgCameraSettings):
    '''
    baxter_core_msgs - CameraSettings
    -
    Used for gettting and writing camera settings on Baxter.

    Data
    -
    - width : `int32`
    - height : `int32`
    - fps : `float32`
    - controls : `array[CameraControl]`
    '''

    # =========
    # Constants
    RES_1280_0800: Tuple[int, int] = (1280, 800)
    RES_0960_0600: Tuple[int, int] = (960, 600)
    RES_0640_0400: Tuple[int, int] = (640, 400)
    RES_0480_0300: Tuple[int, int] = (480, 300)
    RES_0384_0240: Tuple[int, int] = (384, 240)
    RES_0320_0200: Tuple[int, int] = (320, 200)
    VALID_RESOLUTIONS: List[Tuple[int, int]] = [
        RES_1280_0800,
        RES_0960_0600,
        RES_0640_0400,
        RES_0480_0300,
        RES_0384_0240,   
        RES_0320_0200,
    ]

    # ===========
    # Constructor
    def __init__(
            self,
            width: int,
            height: int,
            fps: float,
            controls: array
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        width,
                        self.TYPE_INT32
                    ),
                    self._validate_input(
                        height,
                        self.TYPE_INT32
                    ),
                    self._validate_input(
                        (width, height),
                        self.TYPE_OTHER,
                        options = self.VALID_RESOLUTIONS
                    ),
                    self._validate_input(
                        fps,
                        self.TYPE_FLOAT32
                    ),
                    self._validate_input(
                        controls,
                        self.TYPE_ARRAY
                    )
                ] + [
                    self._validate_input(
                        ctrl,
                        self.TYPE_CAMERACONTROL
                    )
                    for ctrl in controls
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG CameraSettings Construct: {_str}'
                )
            
        # construct instance
        super().__init__()
        self.width = width
        self.height = height
        self.fps = fps
        self.controls = controls

# =======================
# CollisionAvoidanceState
class CollisionAvoidanceState():
    '''
    baxter_core_msgs - AnalogIOState
    '''
    def __init__(self) -> None:
        raise NotImplementedError()

# =======================
# CollisionDetectionState
class CollisionDetectionState():
    '''
    baxter_core_msgs - AnalogIOState
    '''
    def __init__(self) -> None:
        raise NotImplementedError()

# ==============
# DigitalIOState
class DigitalIOState(ROS2_msg, msgDigitalIOState):
    '''
    baxter_core_msgs - DigitalIOState
    -
    Used for reading the state of a DigitalIO on Baxter.

    Data
    -
    - state : `int8`
    - is_input_only : `bool`

    Constants
    -
    - OFF : `int8`
    - ON : `int8`
    - PRESSED : `int8`
    - UNPRESSED : `int8`
    '''

    # =========
    # Constants
    OFF: int = 0
    ON: int = 1
    PRESSED: int = 1
    UNPRESSED: int = 0

    # ===========
    # Constructor
    def __init__(
            self,
            state: int,
            is_input_only: bool
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        state,
                        self.TYPE_INT8,
                        options = [
                            self.OFF,
                            self.ON,
                            self.PRESSED,
                            self.UNPRESSED,
                        ]
                    ),
                    self._validate_input(
                        is_input_only,
                        self.TYPE_BOOL
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG DigitalIOState Construct: {_str}'
                )

        # construct instance
        super().__init__()
        self.state = state
        self.is_input_only = is_input_only

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return {
            'is_input_only': self.is_input_only,
            'state': self.state,
        }

    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgDigitalIOState) -> 'DigitalIOState':
        return DigitalIOState(
            state = msg.state,
            is_input_only = msg.is_input_only
        )

# ===============
# DigitalIOStates
class DigitalIOStates():
    '''
    baxter_core_msgs - AnalogIOState
    '''
    def __init__(self) -> None:
        raise NotImplementedError()

# ====================
# DigitalOutputCommand
class DigitalOutputCommand(ROS2_msg, msgDigitalOutputCommand):
    '''
    baxter_core_msgs - DigitalOutputCommand
    -
    Used for writing a command to a DigitalIO on Baxter.

    Data
    -
    - name : `string`
    - value : `bool`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            name: str,
            value: bool
    ) -> None:
        # validate data
        for _str in (
                [
                    self._validate_input(
                        name,
                        self.TYPE_STRING
                    ),
                    self._validate_input(
                        value,
                        self.TYPE_BOOL
                    )
                ]
        ):
            if _str != '':
                raise ValueError(
                    f'ROS2-MSG DigitalOutputCommand Construct: {_str}'
                )

        # construct instance
        super().__init__()
        self.name = name
        self.value = value

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return {
            'name': self.name,
            'value': self.value
        }

    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgDigitalOutputCommand) -> 'DigitalOutputCommand':
        return DigitalOutputCommand(
            msg.name,
            msg.value
        )

    # ==============
    # Create Message
    def create_msg(self) -> msgDigitalOutputCommand:
        _msg = msgDigitalOutputCommand()
        _msg.name = self.name
        _msg.value = self.value
        return _msg

# ==================
# EndEffectorCommand
class EndEffectorCommand():
    '''
    baxter_core_msgs - AnalogIOState
    '''
    def __init__(self) -> None:
        raise NotImplementedError()

# =====================
# EndEffectorProperties
class EndEffectorProperties():
    '''
    baxter_core_msgs - EndEffectorProperties
    -

    Data
    -
    - id : `int`
    - ui_type : `int`
    - manufacturer : `str`
    - product : `str`
    - serial_number : `str`
    - hardware_rev : `str`
    - firmware_rev : `str`
    - firmware_date : `str`
    - has_calibration : `bool`
    - controls_grip : `bool`
    - senses_grip : `bool`
    - reverses_grip : `bool`
    - controls_force : `bool`
    - senses_force : `bool`
    - controls_position : `bool`
    - senses_position : `bool`
    - properties : `str`

    Constants
    -
    - NO_GRIPPER : `int`
    - SUCTION_CUP_GRIPPER : `int`
    - ELECTRIC_GRIPPER : `int`
    - PASSIVE_GRIPPER : `int`
    '''
    def __init__(self) -> None:
        raise NotImplementedError()

# ================
# EndEffectorState
class EndEffectorState():
    '''
    baxter_core_msgs - EndEffectorState
    -

    Data
    -
    - id : `int`
    - enabled : `int`
    - calibrated : `int`
    - ready : `int`
    - moving : `int`
    - gripping : `int`
    - missed : `int`
    - error : `int`
    - reverse : `int`
    - position : `float`
    - force : `float`
    - state : `str`
    - command : `str`
    - command_sender : `str`
    - command_sequence : `int`

    Constants
    -
    - STATE_FALSE : `int`
    - STATE_TRUE : `int`
    - STATE_UNKNOWN : `int`
    - POSITION_CLOSED : `float`
    - POSITION_OPEN : `float`
    - FORCE_MIN : `float`
    - FORCE_MAX : `float`
    '''
    def __init__(self) -> None:
        raise NotImplementedError()

# =============
# EndpointState
class EndpointState(ROS2_msg, msgEndpointState):
    '''
    baxter_core_msgs - EndPointState
    -

    Data
    -
    - header : `Header`
    - pose : `Pose`
    - twist : `Twist`
    - wrench : `Wrench`
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            header: 'Header',
            pose: 'Pose',
            twist: 'Twist',
            wrench: 'Wrench',
            skip_validation: bool = False
    ) -> None:
        # validate data
        if not skip_validation:
            for _str in (
                    [
                        self._validate_input(
                            header,
                            self.TYPE_HEADER
                        ),
                        self._validate_input(
                            pose,
                            self.TYPE_POSE
                        ),
                        self._validate_input(
                            twist,
                            self.TYPE_TWIST
                        ),
                        self._validate_input(
                            wrench,
                            self.TYPE_WRENCH
                        )
                    ]
            ):
                if _str != '':
                    raise ValueError(
                        f'ROS2-MSG EndpointState Construct: {_str}'
                    )
                
        # construct instance
        super().__init__()
        self.header = header
        self.pose = pose
        self.twist = twist
        self.wrench = wrench

    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgEndpointState) -> 'EndpointState':
        return EndpointState(
            header = Header.from_msg(msg.header),
            pose = Pose.from_msg(msg.pose),
            twist = Twist.from_msg(msg.twist),
            wrench = Wrench.from_msg(msg.wrench),
            skip_validation = True
        )

# ==============
# EndpointStates
class EndpointStates():
    '''
    baxter_core_msgs - EndpointStates
    -

    Data
    -
    names : `list[str]`
    states : `list[EndpointState]`
    '''
    def __init__(self) -> None:
        raise NotImplementedError()

# ==============
# HeadPanCommand
class HeadPanCommand():
    '''
    baxter_core_msgs - HeadPanCommand
    -

    Data
    -
    target : `float`
    speed_ratio : `float`
    enable_pan_request : `int`

    Constants
    -
    - MAX_SPEED_RATIO : `float`
    - MIN_SPEED_RATIO : `float`
    - REQUEST_PAN_DISABLE : `int`
    - REQUEST_PAN_ENABLE : `int`
    - REQUEST_PAN_VOID : `int`
    '''
    def __init__(self) -> None:
        raise NotImplementedError()

# =========
# HeadState
class HeadState():
    '''
    baxter_core_msgs - HeadState
    -

    Data
    -
    - pan : `float`
    - is_turning : `bool`
    - is_nodding : `bool`
    - is_pan_enabled : `bool`
    '''
    def __init__(self) -> None:
        raise NotImplementedError()

# =====
# Image
class Image(ROS2_msg, msgImage):
    '''
    sensor_msgs - Image
    -
    Contains an uncompressed image from one of the cameras on Baxter.

    Data
    -
    - header : `Header`
        - Acquisition time of image.
    - height : `uint32`
        - Number of rows in image.
    - width : `uint32`
        - Number of columns in image.
    - encoding : `string`
        - Pixel encoding.
    - is_bigendian : `uint8`
        - Is the data bigendian.
    - step : `uint32`
        - Full row length in bytes.
    - data : `array[uint8]`
        - Image matrix data, size is (step * rows)
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            header: 'Header',
            height: int,
            width: int,
            encoding: str,
            is_bigendian: int,
            step: int,
            data: array,
            skip_validation: bool = False
    ) -> None:
        # validate data
        if not skip_validation:
            for _str in (
                    [
                        self._validate_input(
                            header,
                            self.TYPE_HEADER
                        ),
                        self._validate_input(
                            height,
                            self.TYPE_UINT32
                        ),
                        self._validate_input(
                            width,
                            self.TYPE_UINT32
                        ),
                        self._validate_input(
                            encoding,
                            self.TYPE_STRING
                        ),
                        self._validate_input(
                            is_bigendian,
                            self.TYPE_UINT8
                        ),
                        self._validate_input(
                            step,
                            self.TYPE_UINT32
                        ),
                        self._validate_input(
                            data,
                            self.TYPE_LIST
                        )
                    ] + [
                        self._validate_input(
                            data_cell,
                            self.TYPE_UINT8
                        )
                        for data_cell in data
                    ]
            ):
                if _str != '':
                    raise ValueError(
                        f'ROS2-MSG Image Construct: {_str}'
                    )
        
        # construct instance
        super().__init__()
        self.header = header
        self.height = height
        self.width = width
        self.encoding = encoding
        self.is_bigendian = is_bigendian
        self.step = step
        self.data = data
        self.channels: int = int(self.step / self.width)

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return {
            True: {
                'header': self.header.frame_id,
                'height': self.height,
                'width': self.width,
            },
            False: {
                'header': self.header,
                'height': self.height,
                'width': self.width,
                'encoding': self.encoding,
                'is_bigendian': self.is_bigendian,
                'step': self.step,
                'data': self.data,
                'CUSTOM: channels': self.channels,
            },
        }[short]
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgImage) -> 'Image':
        return Image(
            header = Header.from_msg(msg.header),
            height = msg.height,
            width = msg.width,
            encoding = msg.encoding,
            is_bigendian = msg.is_bigendian,
            step = msg.step,
            data = msg.data,
            skip_validation = True
        )

# =============
# JointCommand
class JointCommand(ROS2_msg, msgJointCommand):
    '''
    baxter_core_msgs - JointCommand
    -

    Data
    -
    - mode : `int32`
    - command : `float64[]`
    - names : `string[]`

    Constants
    -
    - POSITION_MODE : `int32`
    - VELOCITY_MODE : `int32`
    - TORQUE_MODE : `int32`
    - RAW_POSITION_MODE : `int32`
    '''

    # =========
    # Constants
    POSITION_MODE: int = 1
    VELOCITY_MODE: int = 2
    TORQUE_MODE: int = 3
    RAW_POSITION_MODE: int = 4
    NAME_L_E0: str = 'left_e0'
    NAME_L_E1: str = 'left_e1'
    NAME_L_S0: str = 'left_s0'
    NAME_L_S1: str = 'left_s1'
    NAME_L_W0: str = 'left_w0'
    NAME_L_W1: str = 'left_w1'
    NAME_L_W2: str = 'left_w2'
    NAMES_L: List[str] = [
        NAME_L_E0,
        NAME_L_E1,
        NAME_L_S0,
        NAME_L_S1,
        NAME_L_W0,
        NAME_L_W1,
        NAME_L_W2,
    ]
    NAME_R_E0: str = 'right_e0'
    NAME_R_E1: str = 'right_e1'
    NAME_R_S0: str = 'right_s0'
    NAME_R_S1: str = 'right_s1'
    NAME_R_W0: str = 'right_w0'
    NAME_R_W1: str = 'right_w1'
    NAME_R_W2: str = 'right_w2'
    NAMES_R: List[str] = [
        NAME_R_E0,
        NAME_R_E1,
        NAME_R_S0,
        NAME_R_S1,
        NAME_R_W0,
        NAME_R_W1,
        NAME_R_W2,
    ]
    NAMES_ALL: List[str] = NAMES_L + NAMES_R

    # ===========
    # Constructor
    def __init__(
            self,
            mode: int,
            command: array,
            names: array,
            skip_validation: bool = False
    ) -> None:
        # validate data
        if not skip_validation:
            for _str in (
                    [
                        self._validate_input(
                            mode,
                            self.TYPE_INT32,
                            options = [
                                self.POSITION_MODE,
                                self.VELOCITY_MODE,
                                self.TORQUE_MODE,
                                self.RAW_POSITION_MODE,
                            ]
                        ),
                        self._validate_input(
                            command,
                            self.TYPE_ARRAY
                        ),
                        self._validate_input(
                            names,
                            self.TYPE_ARRAY
                        )
                    ] + [
                        self._validate_input(
                            cmd,
                            self.TYPE_FLOAT64
                        )
                        for cmd in command
                    ] + [
                        self._validate_input(
                            name,
                            self.TYPE_STRING,
                            options = self.NAMES_ALL
                        )
                        for name in names
                    ]
            ):
                if _str != '':
                    raise ValueError(
                        f'ROS2-MSG JointCommand Construct: {_str}'
                    )
                
        # construct instance
        super().__init__()
        self.mode = mode
        self.command = command
        self.names = names

    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgJointCommand) -> 'JointCommand':
        return JointCommand(
            mode = msg.mode,
            command = msg.command,
            names = msg.names,
            skip_validation = True
        )
    
    # ==============
    # Create Message
    def create_msg(self) -> msgJointCommand:
        m = msgJointCommand()
        m.mode = self.mode
        m.command = self.command
        m.names = self.names
        return m

# ==============
# NavigatorState
class NavigatorState():
    '''
    baxter_core_msgs - NavigatorState
    -

    Data
    -
    - button_names : `list[str]`
    - buttons : `list[bool]`
    - wheel : `int`
    - light_names : `list[str]`
    - lights : `list[bool]`
    '''
    def __init__(self) -> None:
        raise NotImplementedError()

# ===============
# NavigatorStates
class NavigatorStates():
    '''
    baxter_core_msgs - NavigatorStates
    -

    Data
    -
    - names : `list[str]`
    - states : `list[NavigatorState]`
    '''
    def __init__(self) -> None:
        raise NotImplementedError()


# =============================================================================
# Custom Messages
# =============================================================================

# ==========
# CameraData
class CameraData(ROS2_msg, msgCameraData):
    '''
    baxter_int_ros2_support - CameraData
    -
    Contains the data required for showing an image from camera data.

    Data
    -
    - image : `array[uint8]`
        - Image Data in 1D array.
    - width : `uint32`
        - Number of columns in image.
    - height : `uint32`
        - Number of rows in image.
    - channels : `uint32`
        - Number of channels per pixel in image.
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            image: array,
            width: int,
            height: int,
            channels: int,
            skip_validation: bool = False
    ) -> None:
        if not skip_validation:
            for _str in (
                    [
                        self._validate_input(
                            image,
                            self.TYPE_ARRAY
                        ),
                        self._validate_input(
                            width,
                            self.TYPE_UINT32
                        ),
                        self._validate_input(
                            height,
                            self.TYPE_UINT32
                        ),
                        self._validate_input(
                            channels,
                            self.TYPE_UINT32
                        ),
                    ] + [
                        self._validate_input(
                            val,
                            self.TYPE_UINT8
                        )
                        for val in image
                    ]
            ):
                if _str != '':
                    raise ValueError(
                        f'ROS2-MSG CameraData Construct: {_str}'
                    )
                
        # construct instance
        super().__init__()
        self.image = image
        self.height = height
        self.width = width
        self.channels = channels

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return { # type: ignore
            True: {
                'width': self.width,
                'height': self.height,
                'channels': self.channels,
            },
            False: {
                'image': self.image,
                'width': self.width,
                'height': self.height,
                'channels': self.channels,
            },
        }[short]
    
    # ===============
    # Create from MSG
    @staticmethod
    def from_msg(msg: msgCameraData) -> 'CameraData':
        return CameraData(
            image = msg.image,
            width = msg.width,
            height = msg.height,
            channels = msg.channels,
            skip_validation = True
        )

    # ==============
    # Create Message
    def create_msg(self) -> msgCameraData:
        _msg = msgCameraData()
        _msg.image = self.image
        _msg.width = self.width
        _msg.height = self.height
        _msg.channels = self.channels
        return _msg


# =============================================================================
# End of File
# =============================================================================
