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
    List,
    Optional,
    TYPE_CHECKING,

    # - rclpy
    Node,
    Publisher,
    spin,
    Subscription,

    # - baxter_core_msgs
    msgEndpointState,
    msgJointCommand,

    # - .dataflow
    df_wait,
    Signal,

    # - .
    Topics,

    # - .msgs
    MSG_EndpointState,
    MSG_JointCommand,
)


# =============================================================================
# Constants Definition
# =============================================================================
ERR_DIR: str = 'baxter_int_ros2.digital_io'


# =============================================================================
# Baxter Digital IO Object
# =============================================================================
class DigitalIO(Node):
    '''
    Baxter Digital IO Object
    -
    Baxter interface object which is able to connect to a single Digital Input
    and/or Output on the Baxter robot.

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
            verbose: bool = False
    ) -> None:
        '''
        Baxter Digital IO Constructor
        -
        Initializes an instance of the Baxter interface object which is able to
        connect to a single Digital Input and/or Output on the Baxter robot.

        Parameters
        -
        - topic : `str`
            - Digital IO topic to subscribe to. Must be in 
                `Topics.DigitalIO.ALL`.
        - verbose : `bool`
            - Whether or not to produce detailed logs about the function.
                Defaults to `False`.

        Returns
        -
        None
        '''

        if verbose: print(f'Constructing DigitalIO, topic={topic}')

        # initialize attributes
        if verbose: print('| - Initializing Variables')
        self._node_name: str
        self._output_enabled: bool
        self._publisher: Publisher
        self._subscriber: Subscription
        self._state: Optional[bool]
        self._topic: str
        self.state_changed: Signal

        # validate the topic
        if verbose: print('| - Validating Topic')
        if not topic in Topics.DigitalIO.ALL:
            raise ValueError(
                f'{ERR_DIR}.DigitalIO.__init__ unable to ' \
                    f'construct object with invalid topic: topic = {topic}, ' \
                    f'type = {type(topic)}'
            )
        
        # create main attributes
        if verbose: print('| - Creating Main Attributes')
        self._topic = topic
        self._output_enabled = False
        self._state = None
        self._node_name = f'BaxterDigitalIO_{topic}'
        self.state_changed = Signal()

        # initialize node
        if verbose: print(f'| - Creating Node (node_name={self._node_name})')
        super().__init__(self._node_name)

        # create subscriber
        if verbose: print(f'| - Creating Subscriber, topic={self.topic}')
        self._subscriber = self.create_subscription(
            MSG_DigitalIOState,
            self.topic,
            self._subscriber_callback,
            10
        )

        # create publisher
        if verbose: print(f'| - Creating Publisher, topic_cmd={self.topic_cmd}')
        self._publisher = self.create_publisher(
            # MSG_DigitalOutputCommand,
            msgDigitalOutputCommand,
            self.topic_cmd,
            10
        )
        
        if verbose: print('| - -----\n| - Done.\n| - -----')
    
    # ===================
    # Custom REPR and STR
    def __repr__(self) -> str:
        return (
            f'DigitalIO(_topic = {self._topic}, topic = {self.topic}, ' \
                + f'topic_cmd = {self.topic_cmd})'
        )
    def __str__(self) -> str:
        return f'DigitalIO<"{self.topic}">'

    # =================
    # Full Topic String
    @property
    def topic(self) -> str:
        ''' Full name of the current `DigitalIO` topic. '''
        return f'/robot/digital_io/{self._topic}/state'
    
    # ====================
    # Command Topic String
    @property
    def topic_cmd(self) -> str:
        ''' Name of the topic used for publishing commands. '''
        return f'/robot/digital_io/command'
    
    # ====================
    # Object Value - State
    @property
    def state(self) -> Optional[bool]:
        ''' Whether or not the `DigitalIO` is pressed. '''
        return self._state
    @state.setter
    def state(self, value: bool) -> None:
        ''' Sets the state of the `DigitalIO`. '''
        self._set_output(value)

    # =============================
    # Object Value - Output Enabled
    @property
    def output_enabled(self) -> bool:
        ''' Whether or not the `DigitalIO` is able to publish data. '''
        return self._output_enabled

    # ===================
    # Subscriber Callback
    def _subscriber_callback(self, msg_original: msgDigitalIOState) -> None:
        '''
        DigitalIO Subscriber Callback
        -
        Runs whenever the `DigitalIO` topic subscriber receives data from the
        topic is it subscribed to.

        Parameters
        -
        - msg_original : `msgDigitalIOState`
            - Contains the data from the `DigitalIO` input.

        Returns
        -
        None
        '''

        # initialize variables
        old_state: Optional[bool]
        msg: MSG_DigitalIOState = MSG_DigitalIOState.from_msg(msg_original)
        new_state: bool

        if self._state is None: # first reading, set output_enabled flag
            self._output_enabled = not msg.is_input_only
        
        # checks if state has changed
        old_state = self._state
        new_state = msg.state == msg.ON
        if (
                (old_state is not None)
                and (old_state != new_state)
        ):
            self.state_changed(new_state)
        self._state = new_state

    # =========
    # Publisher
    def _set_output(
            self,
            value: bool,
            timeout: float = 2.0
    ) -> None:
        '''
        DigitalIO Publisher
        -
        Creates a message to be published in the command topic for the current
        `DigitalIO` object.

        Parameters
        -
        - value : `bool`
            - Value of the state to change the `DigitalIO` to.
        - timeout : `float`
            - Amount of time to wait for the message to get published.

        Returns
        -
        None
        '''

        # validate that the current object can publish
        if not self._output_enabled:
            raise PermissionError(
                f'{ERR_DIR}.DigitalIO trying to set output for {self} but' \
                    + f' _output_enabled = {self._output_enabled}'
            )
        
        # create output command
        cmd = MSG_DigitalOutputCommand(
            name = self._topic,
            value = value
        ).create_msg()
        self._publisher.publish(cmd)

        # attempt to publish until timeout runs out
        if not timeout == 0:
            df_wait(
                lambda: self.state == value,
                self,
                timeout = timeout,
                ros_rate = 100,
                timeout_msg = (
                    f'Failed to publish command {{value = {value}}} for {self}'
                ),
                during_func = lambda: self._publisher.publish(cmd)
            )

    # ==================
    # Node: Destroy Node
    def destroy_node(self) -> None:
        ''' Node: Destroy Node '''
        super().destroy_node()


# =============================================================================
# End of File
# =============================================================================
