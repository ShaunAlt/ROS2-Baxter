#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
ROS2 Baxter Limbs Interfaces
-
Contains object and method definitions required for interacting with the
Limb topics (arm segments) of Baxter.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

from . import (
    # - typing
    Any,
    Callable,
    List,
    Optional,
    TYPE_CHECKING,

    # - array
    array,

    # - rclpy
    Node,
    Publisher,
    spin,
    Subscription,

    # - baxter_core_msgs
    msgEndpointState,
    msgJointCommand,

    # - std_msgs
    msgFloat64,

    # - sensor_msgs
    msgJointState,

    # - .dataflow
    df_wait,
    Signal,

    # - .
    _DATA,
    _JOINT,
    ROS2_Node,
    Topics,

    # - .msgs
    MSG_EndpointState,
    MSG_Float64,
    MSG_JointCommand,
    MSG_JointState,
)


# =============================================================================
# Baxter Limb Object
# =============================================================================
class Limb(ROS2_Node):
    '''
    Baxter Limb Object
    -
    Baxter interface object which is able to connect to a single Limb on the
    Baxter robot.

    Attributes
    -
    None

    Methods
    -
    None
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            topic: str,
            verbose: int = -1
    ) -> None:
        '''
        Baxter Limb Constructor
        -
        Initializes an instance of the Baxter interface object which is able to
        connect to a single Digital Input and/or Output on the Baxter robot.

        Parameters
        -
        - topic : `str`
            - Limb topic to subscribe to. Must be in `Topics.Limb.ALL`.
        - verbose : `int`
            - Defaults to `-1` which means no verbosity. Any value 0 or greater
                represents the number of tabs to indent logs by.

        Returns
        -
        None
        '''

        # validate topic
        if not topic in Topics.Limb.ALL:
            raise ValueError(
                'Limb unable to construct object with invalid topic = ' \
                    f'{repr(topic)}'
            )
        
        # initialize node
        super().__init__(
            f'ROS2_Limb_{topic}',
            verbose
        )
        self.log(f'Constructing Limb - topic={topic}')

        # create main attributes
        self.log('| - Creating Main Attributes')
        self._endpoint: Optional[MSG_EndpointState] = None
        self._joint_e0: Optional[_JOINT] = None
        self._joint_e1: Optional[_JOINT] = None
        self._joint_s0: Optional[_JOINT] = None
        self._joint_s1: Optional[_JOINT] = None
        self._joint_w0: Optional[_JOINT] = None
        self._joint_w1: Optional[_JOINT] = None
        self._joint_w2: Optional[_JOINT] = None
        self._topic: str = topic
        self.state_endpoint = Signal()
        self.state_joint_e0 = Signal()
        self.state_joint_e1 = Signal()
        self.state_joint_s0 = Signal()
        self.state_joint_s1 = Signal()
        self.state_joint_w0 = Signal()
        self.state_joint_w1 = Signal()
        self.state_joint_w2 = Signal()

        # create subcribers
        self.log('| - Creating Subscribers')
        self.log('| - Endpoint State', 1)
        self.create_sub(
            msgEndpointState,
            self.topic_sub_endpoint,
            self.sub_endpoint
        )
        self.create_sub(
            msgJointState,
            self.topic_sub_joints,
            self.sub_joints
        )

        # create publishers
        self.log('| - Creating Publishers')
        self.log('| - Speed Ratio', 1)
        self._pub_speed = self.create_pub(
            msgFloat64,
            self.topic_pub_speed_ratio
        )
        self.log('| - Joint Command', 1)
        self._pub_joint = self.create_pub(
            msgJointCommand,
            self.topic_pub_joint
        )
        self.log('| - Joint Command Timeout', 1)
        self._pub_timeout = self.create_pub(
            msgFloat64,
            self.topic_pub_joint_timeout
        )

        _final_msg: str = str(self)
        if self._V: _final_msg = repr(self)
        self.log(f'| - Created {_final_msg}', override_verbosity = True)

    # ===============
    # Data - Endpoint
    @property
    def data_endpoint(self) -> Optional[MSG_EndpointState]:
        ''' Endpoint state of the `Limb`. '''
        return self._endpoint
    @data_endpoint.setter
    def data_endpoint(self, data: MSG_EndpointState) -> None:
        if (
                (self.data_endpoint is not None)
                and (self.data_endpoint != data)
        ):
            self.state_endpoint(data)
        self._endpoint = data
    
    # ======================
    # Data - Joint - Elbow 0
    @property
    def data_joint_e0(self) -> Optional[_JOINT]:
        ''' `Limb` Elbow 0 Joint: Position, Velocity, Effort. '''
        return self._joint_e0
    @data_joint_e0.setter
    def data_joint_e0(self, data: Optional[_JOINT]) -> None:
        if data is None: return None
        if (self.data_joint_e0 is not None) and (self.data_joint_e0 != data):
            self.state_joint_e0(data)
        self._joint_e0 = data

    # ======================
    # Data - Joint - Elbow 1
    @property
    def data_joint_e1(self) -> Optional[_JOINT]:
        ''' `Limb` Elbow 1 Joint: Position, Velocity, Effort. '''
        return self._joint_e1
    @data_joint_e1.setter
    def data_joint_e1(self, data: Optional[_JOINT]) -> None:
        if data is None: return None
        if (self.data_joint_e1 is not None) and (self.data_joint_e1 != data):
            self.state_joint_e1(data)
        self._joint_e1 = data

    # =========================
    # Data - Joint - Shoulder 0
    @property
    def data_joint_s0(self) -> Optional[_JOINT]:
        ''' `Limb` Shoulder 0 Joint: Position, Velocity, Effort. '''
        return self._joint_s0
    @data_joint_s0.setter
    def data_joint_s0(self, data: Optional[_JOINT]) -> None:
        if data is None: return None
        if (self.data_joint_s0 is not None) and (self.data_joint_s0 != data):
            self.state_joint_s0(data)
        self._joint_s0 = data

    # =========================
    # Data - Joint - Shoulder 1
    @property
    def data_joint_s1(self) -> Optional[_JOINT]:
        ''' `Limb` Shoulder 1 Joint: Position, Velocity, Effort. '''
        return self._joint_s1
    @data_joint_s1.setter
    def data_joint_s1(self, data: Optional[_JOINT]) -> None:
        if data is None: return None
        if (self.data_joint_s1 is not None) and (self.data_joint_s1 != data):
            self.state_joint_s1(data)
        self._joint_s1 = data

    # ======================
    # Data - Joint - Wrist 0
    @property
    def data_joint_w0(self) -> Optional[_JOINT]:
        ''' `Limb` Wrist 0 Joint: Position, Velocity, Effort. '''
        return self._joint_w0
    @data_joint_w0.setter
    def data_joint_w0(self, data: Optional[_JOINT]) -> None:
        if data is None: return None
        if (self.data_joint_w0 is not None) and (self.data_joint_w0 != data):
            self.state_joint_w0(data)
        self._joint_w0 = data

    # ======================
    # Data - Joint - Wrist 1
    @property
    def data_joint_w1(self) -> Optional[_JOINT]:
        ''' `Limb` Wrist 1 Joint: Position, Velocity, Effort. '''
        return self._joint_w1
    @data_joint_w1.setter
    def data_joint_w1(self, data: Optional[_JOINT]) -> None:
        if data is None: return None
        if (self.data_joint_w1 is not None) and (self.data_joint_w1 != data):
            self.state_joint_w1(data)
        self._joint_w1 = data

    # ======================
    # Data - Joint - Wrist 2
    @property
    def data_joint_w2(self) -> Optional[_JOINT]:
        ''' `Limb` Wrist 2 Joint: Position, Velocity, Effort. '''
        return self._joint_w2
    @data_joint_w2.setter
    def data_joint_w2(self, data: Optional[_JOINT]) -> None:
        if data is None: return None
        if (self.data_joint_w2 is not None) and (self.data_joint_w2 != data):
            self.state_joint_w2(data)
        self._joint_w2 = data
        
    # ============
    # Topic - Main
    @property
    def topic(self) -> str:
        ''' Base topic parent for `Limb`. '''
        return f'{Topics.Limb._PREFIX}/{self._topic}'
    
    # ===============================
    # Topic - Publisher - Speed Ratio
    @property
    def topic_pub_speed_ratio(self) -> str:
        ''' Topic used for publishing `speed_ratio` commands to the `Limb`. '''
        return f'{self.topic}/{Topics.Limb._SET_SPEED_RATIO}'

    # =================================
    # Topic - Publisher - Joint Command
    @property
    def topic_pub_joint(self) -> str:
        ''' Topic used for publishing joint commands to the `Limb`. '''
        return f'{self.topic}/{Topics.Limb._SET_JOINT_CMD}'

    # =========================================
    # Topic - Publisher - Joint Command Timeout
    @property
    def topic_pub_joint_timeout(self) -> str:
        ''' 
            Topic used for publishing the timeout for joint commands to the 
            `Limb`.
        '''
        return f'{self.topic}/{Topics.Limb._SET_JOINT_CMD_TIMEOUT}'

    # ===================================
    # Topic - Subscriber - Endpoint State
    @property
    def topic_sub_endpoint(self) -> str:
        ''' Topic used for subscribing to the endpoint state of the `Limb`. '''
        return f'{self.topic}/{Topics.Limb._GET_ENDPOINT}'

    # =================================
    # Topic - Subscriber - Joint States
    @property
    def topic_sub_joints(self) -> str:
        ''' Topic used for subscribing to the joint states of the `Limb`. '''
        return f'{Topics.Limb._GET_JOINT_VALS}'

    # ==============
    # Command Joints
    def _command(
            self,
            cmd_mode: int,
            timeout: float = 0,
            e0: Optional[float] = None,
            e1: Optional[float] = None,
            s0: Optional[float] = None,
            s1: Optional[float] = None,
            w0: Optional[float] = None,
            w1: Optional[float] = None,
            w2: Optional[float] = None,
            dead_zone: float = 0.05,
            tab_increase: int = 0
    ) -> bool:
        '''
        Command Joints
        -
        Used by other methods to command the joints of the `Limb`.

        Parameters
        -
        - cmd_mode : `int`
            - Command mode to run the `MSG_JointCommand` in.
        - timeout : `float`
            - Defaults to `0`, which means the joint command will get published
                once. For any positive value, the command will get looped until
                either the command targets are reached or the timeout is 
                reached.
            - Seconds.
        - e0 : `float | None`
            - Value to give to the Elbow-0 Joint. If `None` then will not
                publish a value to this joint.
        - e1 : `float | None`
            - Value to give to the Elbow-1 Joint. If `None` then will not
                publish a value to this joint.
        - s0 : `float | None`
            - Value to give to the Shoulder-0 Joint. If `None` then will not
                publish a value to this joint.
        - s1 : `float | None`
            - Value to give to the Shoulder-1 Joint. If `None` then will not
                publish a value to this joint.
        - w0 : `float | None`
            - Value to give to the Wrist-0 Joint. If `None` then will not
                publish a value to this joint.
        - w1 : `float | None`
            - Value to give to the Wrist-1 Joint. If `None` then will not
                publish a value to this joint.
        - w2 : `float | None`
            - Value to give to the Wrist-2 Joint. If `None` then will not
                publish a value to this joint.
        - dead_zone : `float`
            - Dead-Zone within which the target goals are allowed to be
                inaccurate. Measured in rad, rad/s, or Nm.
        - tab_increase : `int`
            - Number of tabs to increase the indentation of the command logs
                by.

        Returns
        -
        - `bool`
            - Whether or not the command was successful.
        '''

        # log publish
        self.log(f'* {self.node_name} - Command Joints', tab_increase)

        # validate `cmd_mode`
        if cmd_mode not in MSG_JointCommand.MODES:
            self.log(f'| - Invalid Mode: cmd_mode={cmd_mode}', tab_increase+1)
            return False

        # create command data
        self.log('| - Creating Command', tab_increase+1)
        _names, _cmds = MSG_JointCommand.create_data(
            left = self._topic == Topics.Limb.LEFT,
            e0 = e0,
            e1 = e1,
            s0 = s0,
            s1 = s1,
            w0 = w0,
            w1 = w1,
            w2 = w2
        )
        cmd = MSG_JointCommand(
            mode = cmd_mode,
            command = _cmds, # type: ignore
            names = _names, # type: ignore
            skip_validation = True
        )

        # publish command
        self.log('| - Publishing Command', tab_increase+1)
        self._pub_joint.publish(cmd.create_msg())

        # run timeout loop if required
        if timeout > 0:
            self.log('| - Creating Timeout Target Function', tab_increase+1)
            _idx: int = {
                MSG_JointCommand.POSITION_MODE: 0,
                MSG_JointCommand.VELOCITY_MODE: 1,
                MSG_JointCommand.TORQUE_MODE: 2,
                MSG_JointCommand.RAW_POSITION_MODE: 0,
            }[cmd_mode]
            _flag_func: Callable[[], bool] = lambda: (
                (
                    (e0 is None)
                    or (
                        (self.data_joint_e0 is not None)
                        and (self.data_joint_e0[_idx] >= e0 - dead_zone)
                        and (self.data_joint_e0[_idx] <= e0 + dead_zone)
                    )
                )
                and (
                    (e1 is None)
                    or (
                        (self.data_joint_e1 is not None)
                        and (self.data_joint_e1[_idx] >= e1 - dead_zone)
                        and (self.data_joint_e1[_idx] <= e1 + dead_zone)
                    )
                )
                and (
                    (s0 is None)
                    or (
                        (self.data_joint_s0 is not None)
                        and (self.data_joint_s0[_idx] >= s0 - dead_zone)
                        and (self.data_joint_s0[_idx] <= s0 + dead_zone)
                    )
                )
                and (
                    (s1 is None)
                    or (
                        (self.data_joint_s1 is not None)
                        and (self.data_joint_s1[_idx] >= s1 - dead_zone)
                        and (self.data_joint_s1[_idx] <= s1 + dead_zone)
                    )
                )
                and (
                    (w0 is None)
                    or (
                        (self.data_joint_w0 is not None)
                        and (self.data_joint_w0[_idx] >= w0 - dead_zone)
                        and (self.data_joint_w0[_idx] <= w0 + dead_zone)
                    )
                )
                and (
                    (w1 is None)
                    or (
                        (self.data_joint_w1 is not None)
                        and (self.data_joint_w1[_idx] >= w1 - dead_zone)
                        and (self.data_joint_w1[_idx] <= w1 + dead_zone)
                    )
                )
                and (
                    (w2 is None)
                    or (
                        (self.data_joint_w2 is not None)
                        and (self.data_joint_w2[_idx] >= w2 - dead_zone)
                        and (self.data_joint_w2[_idx] <= w2 + dead_zone)
                    )
                )
            )
            self.log('| - Ensuring Joint Goals are Met', tab_increase+1)
            return df_wait(
                _flag_func,
                self,
                timeout,
                raise_err = False,
                timeout_msg = 'Limb {self.node_name} Joint Command Timeout',
                during_func = lambda: self._pub_joint.publish(cmd.create_msg())
            )

        # log publish success
        self.log('| - Done', tab_increase+1)
        return True
    
    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        if short:
            return {
                'name': self.node_name,
                'topic': self.topic,
            }
        return {
            'node_name': self.node_name,
            'topic': self.topic,
            'topic_pub_speed_ratio': self.topic_pub_speed_ratio,
            'topic_pub_joint': self.topic_pub_joint,
            'topic_pub_joint_timeout': self.topic_pub_joint_timeout,
            'topic_sub_endpoint': self.topic_sub_endpoint,
            'topic_sub_joints': self.topic_sub_joints,
            'data_endpoint': self.data_endpoint,
            'data_joint_e0': self.data_joint_e0,
            'data_joint_e1': self.data_joint_e1,
            'data_joint_s0': self.data_joint_s0,
            'data_joint_s1': self.data_joint_s1,
            'data_joint_w0': self.data_joint_w0,
            'data_joint_w1': self.data_joint_w1,
            'data_joint_w2': self.data_joint_w2,
        }
    
    # =====================
    # Publish - Speed Ratio
    def pub_speed(
            self,
            speed: float
    ) -> None:
        '''
        Publish - Speed Ratio
        -
        Publishes a desired speed ratio to the `Limb`.

        Parameters
        -
        - speed : `float`
            - Desired speed ratio.

        Returns
        -
        None
        '''

        # log publish
        self.log(f'* {self.node_name}')
        self.log('| - Publishing Speed Ratio', 1)

        # publish data
        self._pub_speed.publish(MSG_Float64(speed).create_msg())

        # log publish success
        self.log(f'| - Set Speed Ratio to {speed}', 1)
        self.log('| - Done', 1)

    # =======================
    # Publish - Joint Timeout
    def pub_timeout(
            self,
            timeout: float
    ) -> None:
        '''
        Publish - Joint Timeout
        -
        Publishes a specific joint command timeout to the `Limb`.

        Parameters
        -
        - timeout : `float`
            - Desired timeout value.

        Returns
        -
        None
        '''

        # log publish
        self.log(f'* {self.node_name}')
        self.log('| - Publishing Joint Timeout', 1)

        # publish data
        self._pub_timeout.publish(MSG_Float64(timeout).create_msg())

        # log publish success
        self.log(f'| - Set Joint Timeout to {timeout}', 1)
        self.log('| - Done', 1)
    
    # ===================
    # Set Joint Positions
    def set_positions(
            self,
            timeout: float = 0,
            e0: Optional[float] = None,
            e1: Optional[float] = None,
            s0: Optional[float] = None,
            s1: Optional[float] = None,
            w0: Optional[float] = None,
            w1: Optional[float] = None,
            w2: Optional[float] = None,
            dead_zone: float = 0.05,
            tab_increase: int = 0
    ) -> bool:
        '''
        Set Joint Positions
        -
        Sets the positions of the `Limb` joints.

        Parameters
        -
        - timeout : `float`
            - Defaults to `0`, which means the joint command will get published
                once. For any positive value, the command will get looped until
                either the command targets are reached or the timeout is 
                reached.
            - Seconds.
        - e0 : `float | None`
            - Value to give to the Elbow-0 Joint. If `None` then will not
                publish a value to this joint.
        - e1 : `float | None`
            - Value to give to the Elbow-1 Joint. If `None` then will not
                publish a value to this joint.
        - s0 : `float | None`
            - Value to give to the Shoulder-0 Joint. If `None` then will not
                publish a value to this joint.
        - s1 : `float | None`
            - Value to give to the Shoulder-1 Joint. If `None` then will not
                publish a value to this joint.
        - w0 : `float | None`
            - Value to give to the Wrist-0 Joint. If `None` then will not
                publish a value to this joint.
        - w1 : `float | None`
            - Value to give to the Wrist-1 Joint. If `None` then will not
                publish a value to this joint.
        - w2 : `float | None`
            - Value to give to the Wrist-2 Joint. If `None` then will not
                publish a value to this joint.
        - dead_zone : `float`
            - Dead-Zone within which the target goals are allowed to be
                inaccurate. Measured in rad.
        - tab_increase : `int`
            - Number of tabs to increase the indentation of the command logs
                by.

        Returns
        -
        - `bool`
            - Whether or not the command was successful.
        '''

        # log publish
        self.log(f'* {self.node_name} - Set Joint Positions', tab_increase)

        # run command
        self.log(f'| - Running Position Command', tab_increase+1)
        _res = self._command(
            MSG_JointCommand.POSITION_MODE,
            timeout,
            e0,
            e1,
            s0,
            s1,
            w0,
            w1,
            w2,
            dead_zone,
            tab_increase+1
        )

        if _res:
            self.log('| - Done', tab_increase+1)
            return True
        
        self.log('| - Unable to Complete Action', tab_increase+1)
        return False

    # ====================
    # Set Joint Velocities
    def set_velocities(
            self,
            timeout: float = 0,
            e0: Optional[float] = None,
            e1: Optional[float] = None,
            s0: Optional[float] = None,
            s1: Optional[float] = None,
            w0: Optional[float] = None,
            w1: Optional[float] = None,
            w2: Optional[float] = None,
            dead_zone: float = 0.05,
            tab_increase: int = 0
    ) -> bool:
        '''
        Set Joint Velocities
        -
        Sets the velocities of the `Limb` joints.

        Parameters
        -
        - timeout : `float`
            - Defaults to `0`, which means the joint command will get published
                once. For any positive value, the command will get looped until
                either the command targets are reached or the timeout is 
                reached.
            - Seconds.
        - e0 : `float | None`
            - Value to give to the Elbow-0 Joint. If `None` then will not
                publish a value to this joint.
        - e1 : `float | None`
            - Value to give to the Elbow-1 Joint. If `None` then will not
                publish a value to this joint.
        - s0 : `float | None`
            - Value to give to the Shoulder-0 Joint. If `None` then will not
                publish a value to this joint.
        - s1 : `float | None`
            - Value to give to the Shoulder-1 Joint. If `None` then will not
                publish a value to this joint.
        - w0 : `float | None`
            - Value to give to the Wrist-0 Joint. If `None` then will not
                publish a value to this joint.
        - w1 : `float | None`
            - Value to give to the Wrist-1 Joint. If `None` then will not
                publish a value to this joint.
        - w2 : `float | None`
            - Value to give to the Wrist-2 Joint. If `None` then will not
                publish a value to this joint.
        - dead_zone : `float`
            - Dead-Zone within which the target goals are allowed to be
                inaccurate. Measured in rad/s.
        - tab_increase : `int`
            - Number of tabs to increase the indentation of the command logs
                by.

        Returns
        -
        - `bool`
            - Whether or not the command was successful.
        '''

        # log publish
        self.log(f'* {self.node_name} - Set Joint Velocities', tab_increase)

        # run command
        self.log(f'| - Running Velocity Command', tab_increase+1)
        _res = self._command(
            MSG_JointCommand.VELOCITY_MODE,
            timeout,
            e0,
            e1,
            s0,
            s1,
            w0,
            w1,
            w2,
            dead_zone,
            tab_increase+1
        )

        if _res:
            self.log('| - Done', tab_increase+1)
            return True
        
        self.log('| - Unable to Complete Action', tab_increase+1)
        return False

    # =================
    # Set Joint Torques
    def set_torques(
            self,
            timeout: float = 0,
            e0: Optional[float] = None,
            e1: Optional[float] = None,
            s0: Optional[float] = None,
            s1: Optional[float] = None,
            w0: Optional[float] = None,
            w1: Optional[float] = None,
            w2: Optional[float] = None,
            dead_zone: float = 0.05,
            tab_increase: int = 0
    ) -> bool:
        '''
        Set Joint Torques
        -
        Sets the torques of the `Limb` joints.

        Parameters
        -
        - timeout : `float`
            - Defaults to `0`, which means the joint command will get published
                once. For any positive value, the command will get looped until
                either the command targets are reached or the timeout is 
                reached.
            - Seconds.
        - e0 : `float | None`
            - Value to give to the Elbow-0 Joint. If `None` then will not
                publish a value to this joint.
        - e1 : `float | None`
            - Value to give to the Elbow-1 Joint. If `None` then will not
                publish a value to this joint.
        - s0 : `float | None`
            - Value to give to the Shoulder-0 Joint. If `None` then will not
                publish a value to this joint.
        - s1 : `float | None`
            - Value to give to the Shoulder-1 Joint. If `None` then will not
                publish a value to this joint.
        - w0 : `float | None`
            - Value to give to the Wrist-0 Joint. If `None` then will not
                publish a value to this joint.
        - w1 : `float | None`
            - Value to give to the Wrist-1 Joint. If `None` then will not
                publish a value to this joint.
        - w2 : `float | None`
            - Value to give to the Wrist-2 Joint. If `None` then will not
                publish a value to this joint.
        - dead_zone : `float`
            - Dead-Zone within which the target goals are allowed to be
                inaccurate. Measured in Nm.
        - tab_increase : `int`
            - Number of tabs to increase the indentation of the command logs
                by.

        Returns
        -
        - `bool`
            - Whether or not the command was successful.
        '''

        # log publish
        self.log(f'* {self.node_name} - Set Joint Torques', tab_increase)

        # run command
        self.log(f'| - Running Torque Command', tab_increase+1)
        _res = self._command(
            MSG_JointCommand.TORQUE_MODE,
            timeout,
            e0,
            e1,
            s0,
            s1,
            w0,
            w1,
            w2,
            dead_zone,
            tab_increase+1
        )

        if _res:
            self.log('| - Done', tab_increase+1)
            return True
        
        self.log('| - Unable to Complete Action', tab_increase+1)
        return False

    # ====================================
    # Subscriber Callback - Endpoint State
    def sub_endpoint(
            self,
            _msg: msgEndpointState
    ) -> None:
        '''
        Subscriber Callback - Endpoint State
        -
        Runs whenever data is published to the `topic_sub_endpoint` topic.

        Parameters
        -
        - _msg : `msgEndpointState`
            - Contains the position and orientation data of the `Limb`
                end-effector.

        Returns
        -
        None
        '''

        # get data
        msg = MSG_EndpointState.from_msg(_msg)

        # update endpoint data
        self.data_endpoint = msg

    # ==================================
    # Subscriber Callback - Joint States
    def sub_joints(
            self,
            _msg: msgJointState
    ) -> None:
        '''
        Subscriber Callback - Joint States
        -
        Runs whenever data is published to the `topic_sub_joints` topic.

        Parameters
        -
        - _msg : `msgJointState`
            - Contains the position, velocity, and effort data for each of the
                joints in the `Limb`.

        Returns
        -
        None
        '''

        # get data
        msg = MSG_JointState.from_msg(_msg)

        # update joint data
        if self._topic == Topics.Limb.LEFT: # left arm
            self.data_joint_e0 = msg.get_val(MSG_JointState.NAME_LEFT_E0)
            self.data_joint_e1 = msg.get_val(MSG_JointState.NAME_LEFT_E1)
            self.data_joint_s0 = msg.get_val(MSG_JointState.NAME_LEFT_S0)
            self.data_joint_s1 = msg.get_val(MSG_JointState.NAME_LEFT_S1)
            self.data_joint_w0 = msg.get_val(MSG_JointState.NAME_LEFT_W0)
            self.data_joint_w1 = msg.get_val(MSG_JointState.NAME_LEFT_W1)
            self.data_joint_w2 = msg.get_val(MSG_JointState.NAME_LEFT_W2)
        else: # right arm
            self.data_joint_e0 = msg.get_val(MSG_JointState.NAME_RIGHT_E0)
            self.data_joint_e1 = msg.get_val(MSG_JointState.NAME_RIGHT_E1)
            self.data_joint_s0 = msg.get_val(MSG_JointState.NAME_RIGHT_S0)
            self.data_joint_s1 = msg.get_val(MSG_JointState.NAME_RIGHT_S1)
            self.data_joint_w0 = msg.get_val(MSG_JointState.NAME_RIGHT_W0)
            self.data_joint_w1 = msg.get_val(MSG_JointState.NAME_RIGHT_W1)
            self.data_joint_w2 = msg.get_val(MSG_JointState.NAME_RIGHT_W2)


# =============================================================================
# End of File
# =============================================================================
