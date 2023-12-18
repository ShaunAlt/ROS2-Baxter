#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
ROS2 Baxter Gripper Interface
-
Contains object and method definitions required for interacting with the
Gripper topics (digital buttons and sensors) of Baxter.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

from . import (
    # - typing
    Any,
    cast,
    Callable,
    Dict,
    List,
    Optional,
    Tuple,
    TYPE_CHECKING,
    Union,

    # - numpy
    numpy,

    # - array
    array,

    # - opencv
    cv2,
    MatLike,

    # - json
    JSONEncoder,

    # - time
    time,

    # - traceback
    traceback,

    # - rclpy
    Publisher,
    spin_until_future_complete,
    Timer,

    # - baxter_core_msgs
    msgEndEffectorCommand,
    msgEndEffectorProperties,
    msgEndEffectorState,

    # - baxter_int_ros2_support
    msgCameraData,

    # - .dataflow
    df_wait,
    Signal,

    # - .
    _DATA,
    _TIMER,
    ROS2_Node,
    Settings,
    Topics,

    # - .msgs
    MSG_EndEffectorCommand,
    MSG_EndEffectorProperties,
    MSG_EndEffectorState,
)


# =============================================================================
# Baxter Gripper Object
# =============================================================================
class Gripper(ROS2_Node):
    '''
    Baxter Gripper Object
    -
    Baxter interface object which is able to connect to a single Gripper on the
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
        Baxter Gripper Constructor
        -
        Initializes an instance of the Baxter interface object which is able to
        connect to a single gripper on the baxter robot.

        Parameters
        -
        - topic : `str`
            - Gripper topic to subscribe to. Must be in `Topics.Gripper.ALL`.
        - verbose : `int`
            - Defaults to `-1` which means no verbosity. Any value 0 or greater
                represents the number of tabs to indent logs by.

        Returns
        -
        None
        '''

        # validate the topic
        if not topic in Topics.Gripper.ALL:
            raise ValueError(
                'Gripper unable to construct object with invalid' \
                    + f' topic: topic = {repr(topic)}'
            )
        
        # initialize node
        super().__init__(
            f'ROS2_Gripper_{topic}',
            verbose
        )
        _t = '\t' * self._verbose
        if self._V: print(f'{_t}Constructing Gripper - topic={topic}')

        # create main attributes
        if self._V: print(f'{_t}| - Creating Main Attributes')
        self._cmd_seq: int = 0
        self._data_prop: Optional[MSG_EndEffectorProperties] = None
        self._data_state: Optional[MSG_EndEffectorState] = None
        self._topic: str = f'{topic}{Topics.Gripper.SubTopics.SUFFIX}'
        self.state_gripping = Signal()
        self.state_moving = Signal()
        self.state_type = Signal()

        # create subscribers
        if self._V: print(f'{_t}| - Creating Subscribers')
        if self._V: print(f'{_t}\t| - Properties')
        self.create_sub(
            msgEndEffectorProperties,
            self.topic_sub_props,
            self.sub_props
        )
        if self._V: print(f'{_t}\t| - States')
        self.create_sub(
            msgEndEffectorState,
            self.topic_sub_state,
            self.sub_state
        )

        # create publishers
        if self._V: print(f'{_t}| - Creating Publishers')
        if self._V: print(f'{_t}\t| - Command')
        self._pub_cmd = self.create_pub(
            msgEndEffectorCommand,
            self.topic_pub_cmd
        )
        if self._V: print(f'{_t}\t| - Properties')
        self._pub_props = self.create_pub(
            msgEndEffectorProperties,
            self.topic_pub_props
        )
        if self._V: print(f'{_t}\t| - State')
        self._pub_state = self.create_pub(
            msgEndEffectorState,
            self.topic_pub_state
        )

        if self._V:
            print(
                f'{_t}| - Finished Creating:\n{_t}\t' \
                    + repr(self).replace('\n', f'\n\t{_t}')
            )
        else:
            print(f'Created {self}')

    # =======================
    # Command Sequence Number
    @property
    def cmd_seq(self) -> int:
        ''' Increments and returns the Command Sequence Number. '''
        self._cmd_seq += 1
        return self._cmd_seq

    # =================
    # Data - Properties
    @property
    def data_props(self) -> Optional[MSG_EndEffectorProperties]:
        ''' Properties data for `Gripper`. '''
        return self._data_prop
    
    # ============
    # Data - State
    @property
    def data_state(self) -> Optional[MSG_EndEffectorState]:
        ''' State data for `Gripper`. '''
        return self._data_state

    # ============
    # Topic - Main
    @property
    def topic(self) -> str:
        ''' Base topic parent for gripper. '''
        return f'{Topics.Gripper.SubTopics.PREFIX}/{self._topic}'
    
    # =======================
    # Topic - Publish Command
    @property
    def topic_pub_cmd(self) -> str:
        ''' Topic used for publishing commands to the `Gripper`. '''
        return f'{self.topic}/{Topics.Gripper.SubTopics.SET_CMD}'

    # ==========================
    # Topic - Publish Properties
    @property
    def topic_pub_props(self) -> str:
        ''' Topic used for publishing properties of the `Gripper`. '''
        return f'{self.topic}/{Topics.Gripper.SubTopics.SET_PROPS}'
    
    # =====================
    # Topic - Publish State
    @property
    def topic_pub_state(self) -> str:
        ''' Topic used for publishing states of the `Gripper`. '''
        return f'{self.topic}/{Topics.Gripper.SubTopics.SET_STATE}'
    
    # ============================
    # Topic - Subscribe Properties
    @property
    def topic_sub_props(self) -> str:
        ''' Topic used for subscribing to properties of the `Gripper`. '''
        return f'{self.topic}/{Topics.Gripper.SubTopics.GET_PROPS}'

    # =======================
    # Topic - Subscribe State
    @property
    def topic_sub_state(self) -> str:
        ''' Topic used for subscribing to states of the `Gripper`. '''
        return f'{self.topic}/{Topics.Gripper.SubTopics.GET_STATE}'
    
    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return cast(
            _DATA,
            {
                True: {
                    'node_name': self.node_name,
                    'topic': self.topic,
                },
                False: {
                    'node_name': self.node_name,
                    'topic': self.topic,
                    'topic_pub_cmd': self.topic_pub_cmd,
                    'topic_pub_props': self.topic_pub_props,
                    'topic_pub_state': self.topic_pub_state,
                    'topic_sub_props': self.topic_sub_props,
                    'topic_sub_state': self.topic_sub_state,
                    'data_props': self.data_props,
                    'data_state': self.data_state,
                },
            }[short]
        )

    # ==============
    # Create Command
    def _command(
            self,
            cmd: str,
            timeout: float = 2.0,
            block_test: Callable[[], bool] = lambda: True,
            args: Optional[Dict[str, Any]] = None
    ) -> None:
        '''
        Create Command
        -
        Creates a command for the `Gripper`.

        Parameters
        -
        - cmd : `str`
            - Command being published to the `Gripper`.
        - timeout : `float`
            - Number of seconds after which to timeout the flag function and 
                raise an error. Defaults to 0 seconds, which means no 
                timeout block (the function will run once and then the program
                will move on).
        - block_test : `Callable[[], bool]`
            - Test function to be used in the timeout block.
        - args : `dict[str, Any] | None`
            - Optional arguments to parse to the command.

        Returns
        -
        None
        '''

        # set verbosity indentation
        _t: str = '\t' * self._verbose_sub
        if self._V: print(f'{_t}* Create Command ({self.node_name} - {cmd})')

        # validate state data exists
        if self.data_state is None:
            raise RuntimeWarning(
                f'Gripper {self} tried command without any `data_state` set.'
            )

        # create arguments
        if self._V: print(f'{_t}| - Creating Args')
        _a: str
        if args is not None:
            _a = JSONEncoder().encode(args)
        else:
            _a = JSONEncoder().encode({
                'position': 50,
                'velocity': 30,
                'moving_force': 30,
                'holding_force': 30,
                'dead_zone': 5,
            })

        # create command
        if self._V: print(f'{_t}| - Creating Command')
        _cmd = MSG_EndEffectorCommand(
            id = self.data_state.id,
            command = cmd,
            args = _a,
            sender = self.node_name,
            sequence = self.cmd_seq
        )

        # publish command
        if self._V: print(f'{_t}| - Publishing Command')
        self._pub_cmd.publish(_cmd.create_msg())

        if timeout > 0:
            # ensure message was acknowledged
            if self._V: print(f'{_t}| - Ensure Message Acknowledgement')
            msg_ack = df_wait(
                lambda: (
                    (self.data_state is not None)
                    and (self.data_state.command_sender == _cmd.sender) 
                    and (
                        (self.data_state.command_sequence == _cmd.sequence)
                        or (self.data_state.command_sequence == 0)
                    )
                ),
                self,
                timeout,
                raise_err = False,
                timeout_msg = f'{self} Command {cmd} Timeout',
                during_func = lambda: self._pub_cmd.publish(_cmd.create_msg())
            )
            if not msg_ack:
                raise RuntimeWarning(
                    f'Gripper {self.node_name} Command {cmd} not Acknowledged.'
                )
            
            # ensure message was run as required
            if self._V: print(f'{_t}| - Ensure Message Completed')
            df_wait(
                block_test,
                self,
                timeout,
                during_func = lambda: self._pub_cmd.publish(_cmd.create_msg())
            )

    # =========
    # Calibrate
    def calibrate(
            self,
            clear_calibration: bool = False,
            verbosity: int = -1
    ) -> None:
        '''
        Calibrate
        -
        Calibrates the gripper setting the maximum and minimum travel
        distances.

        Parameters
        -
        - clear_calibration : `bool`
            - Whether or not to to clear the current gripper calibration.
        - verbosity : `int`
            - Override value which overrides the current verbosity level for
                the `Gripper` object.

        Returns
        -
        None
        '''

        # setting verbosity indentation
        V: bool = self._V or verbosity >= 0
        _t: str = '\t' * max(self._verbose_sub, verbosity)
        _sub_v: int = {
            True: max(self._verbose_sub, verbosity) + 1,
            False: -1
        }[V]
        if V: print(f'{_t}* Calibrate {self.node_name}')

        # validate state data exists
        if self.data_state is None:
            raise RuntimeWarning(
                f'Gripper {self} tried calibrate without any `data_state` set.'
            )
        
        # create command
        if clear_calibration:
            if V: print(f'{_t}| - Clear Calibration')
            self._command(
                MSG_EndEffectorCommand.CMD_CLEAR_CALIBRATION,
                block_test = lambda: (
                    (self.data_state is not None)
                    and (
                        self.data_state.calibrated \
                            == MSG_EndEffectorState.STATE_FALSE
                    )
                )
            )
        else:
            if V: print(f'{_t}| - Calibrate')
            if self.data_state.calibrated == self.data_state.STATE_TRUE:
                if V: print(f'{_t}| - Clear Calibration First')
                self.calibrate(clear_calibration = True, verbosity = _sub_v)
            self._command(
                MSG_EndEffectorCommand.CMD_CALIBRATE,
                block_test = lambda: (
                    (self.data_state is not None)
                    and (
                        self.data_state.calibrated \
                            == MSG_EndEffectorState.STATE_TRUE
                    )
                )
            )
        
        if V: print(f'{_t}| - Done')

    # =========
    # Configure
    def configure(self) -> None:
        '''
        Configure
        -
        Configures the gripper.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # validate state data exists
        if self.data_state is None:
            raise RuntimeWarning(
                f'Gripper {self} tried calibrate without any `data_state` set.'
            )

        # publish configuration command
        self._command(
            MSG_EndEffectorCommand.CMD_CONFIGURE
        )

    # =====
    # Reset
    def reset(self) -> None:
        '''
        Reset
        -
        Resets the gripper state, removing any errors.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # validate state data exists
        if self.data_state is None:
            raise RuntimeWarning(
                f'Gripper {self} tried reset without any `data_state` set.'
            )
        
        # create command
        self._command(
            MSG_EndEffectorCommand.CMD_RESET
        )

    # ============
    # Set Position
    def set_pos(
            self,
            pos: float,
            block: bool = True,
            velocity: float = 50.0,
            moving_force: float = 50.0,
            holding_force: float = 50.0,
            dead_zone: float = 1.0
    ) -> None:
        '''
        Set Position
        -
        Command the gripper to go to a particular position.

        Parameters
        -
        - pos : `float`
            - Position to move the gripper to. 0 = Closed, 100 = Open.
        - block : `bool`
            - Whether to use a block command or not. Block commands mean that
                the gripper will ignore all other commands until the action has
                been completed.
            - NOT IN USE.
        - velocity : `float`
            - Velocity at which the position move will be executed. Defaults to
                50.0 (half-speed).
        - moving_force : `float`
            - Force threshold at which a move will stop. Defaults to 50.0 (half
                force).
        - holding_force : `float`
            - Force threshold at which a static grasp will continue holding.
                Defaults to 50.0 (half force).
        - dead_zone : `float`
            - Position dead-band within which a position move is considered
                successful. Defaults to 1.0 which is the equivalent of 1%
                position deadband.
        '''

        # validate state data exists
        if self.data_state is None:
            raise RuntimeWarning(
                f'Gripper {self} tried set_pos without any `data_state` set.'
            )
        
        # validate gripper is calibrated
        if self.data_state.calibrated != MSG_EndEffectorState.STATE_TRUE:
            raise RuntimeWarning(
                f'Gripper {self} tried set_pos but Gripper is not calibrated.'
            )
        
        # thresholding values
        pos = max(min(pos, 100), 0)
        velocity = max(min(velocity, 100), 0)
        moving_force = max(min(moving_force, 100), 0)
        holding_force = max(min(holding_force, 100), 0)
        dead_zone = max(min(dead_zone, 20), 0)
        
        # create command
        self._command(
            MSG_EndEffectorCommand.CMD_GO,
            args = {
                'position': pos,
                'velocity': velocity,
                'moving_force': moving_force,
                'holding_force': holding_force,
                'dead_zone': dead_zone,
            },
            block_test = lambda: (
                (self.data_state is not None)
                and (self.data_state.position >= pos - dead_zone)
                and (self.data_state.position <= pos + dead_zone)
            )
        )

    # ================================
    # Subscriber Callback - Properties
    def sub_props(
            self,
            msg: msgEndEffectorProperties
    ) -> None:
        '''
        Subscriber Callback - `EndEffectorProperties`
        -
        Runs whenever data is published to the `topic_sub_props` topic.

        Parameters
        -
        - msg : `msgEndEffectorProperties`
            - Contains the data from the `msgEndEffectorProperties` mesasge.

        Returns
        -
        None
        '''

        # check if this was the first subscriber data pull
        _first_check: bool = self.data_props is None

        # get data
        _msg = MSG_EndEffectorProperties.from_msg(msg)

        # check if gripper type has changed
        _type_change: bool = (
            (self.data_props is not None)
            and (self.data_props.ui_type != _msg.ui_type)
        )

        # set data
        self._data_prop = _msg

        # complete version check if required
        if _first_check or _type_change:
            self.version_check()

        # note gripper type state change
        if _type_change:
            self.state_type(_msg.ui_type)

    # ===========================
    # Subscriber Callback - State
    def sub_state(
            self,
            msg: msgEndEffectorState
    ) -> None:
        '''
        Subscriber Callback - `EndEffectorState`
        -
        Runs whenever data is published to the `topic_sub_state` topic.

        Parameters
        -
        - msg : `msgEndEffectorState`
            - Contains the data from the `msgEndEffectorState` mesasge.

        Returns
        -
        None
        '''

        # check if this is the first subscriber data pull
        _first_check: bool = self.data_state is None

        # get data
        _msg = MSG_EndEffectorState.from_msg(msg)

        # check if gripping state has changed
        _grip_change: bool = (
            (self.data_state is not None)
            and (self.data_state.gripping != _msg.gripping)
        )
        _move_change: bool = (
            (self.data_state is not None)
            and (self.data_state.moving != _msg.moving)
        )

        # set data
        self._data_state = _msg

        # note gripping state change
        if _grip_change:
            self.state_gripping(
                _msg.gripping == MSG_EndEffectorState.STATE_TRUE
            )
        if _move_change:
            self.state_moving(
                _msg.moving == MSG_EndEffectorState.STATE_TRUE
            )

    # =============
    # Version Check
    def version_check(self) -> bool:
        '''
        Version Check
        -
        Completes a safety check on the firmware build date of electric
        grippers with the SDK software version.

        Parameters
        -
        None

        Returns
        -
        `bool`
            - True if gripper version is compatible. False if fail.
        '''

        # make sure property data exists
        if self.data_props is None:
            raise RuntimeWarning(
                f'Gripper {self} tried to version_check with no Property data.'
            )
        
        # only version-check electric grippers
        if not self.data_props.electric:
            return True
        
        # make sure firmware date data is valid
        if self.data_props.firmware_date in ['', None]:
            raise RuntimeWarning(
                f'Gripper {self} version_check couldnt get firmware_data ' \
                    + 'properties.'
            )
        
        # validate firmware date
        _dt_format: str = r'%Y/%m/%d %H:%M:%S'
        dt_firmware: float = time.mktime(time.strptime(
            cast(str, self.data_props.firmware_date),
            _dt_format
        ))
        dt_warn: float = time.mktime(time.strptime(
            Settings.VERSIONS_SDK2GRIPPER[Settings.SDK_VERSION]['warn'],
            _dt_format
        ))
        dt_fail: float = time.mktime(time.strptime(
            Settings.VERSIONS_SDK2GRIPPER[Settings.SDK_VERSION]['fail'],
            _dt_format
        ))
        if dt_firmware > dt_warn:
            return True
        elif dt_firmware > dt_fail:
            raise RuntimeWarning(
                f'Gripper {self} firmware version WARNING.'
            )
        else:
            raise RuntimeError(
                f'Gripper {self} firmware version FAIL.'
            )


# =============================================================================
# End of File
# =============================================================================
