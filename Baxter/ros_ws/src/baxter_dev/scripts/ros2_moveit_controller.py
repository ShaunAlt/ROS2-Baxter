#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter MoveIT ROS2 Receiver Controller
-
Receives the data published from ROS2 that was bridged into ROS1, and then
plans and executes the motion defined by MoveIT to move the Baxter robot.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# ROS Messages - Custom Baxter Interface
from baxter_interface_msgs.msg import (
    EndpointTarget,
    EndpointTargets,
)

# used for date/time
from datetime import (
    datetime,
)

# ROS Messages - Geometry
from geometry_msgs.msg import ( # type: ignore
    Point,
    Pose,
    Quaternion,
)

# MoveIT Commander
import moveit_commander # type: ignore

# ROS Messages - MoveIT
from moveit_msgs.msg import ( # type: ignore
    RobotTrajectory,
)

# ROS1-Python Package
import rospy # type: ignore

# used for system arguments
import sys

# used for multi-threading
import threading

# used for type hinting
from typing import (
    Any,
    Dict,
    List,
    Optional,
    Tuple,
    TYPE_CHECKING,
)

if TYPE_CHECKING:
    # Baxter Interface Objects
    from baxter_legacy.baxter_interface.src.baxter_interface import (
        Limb,
    )
else:
    # Baxter Interface Objects
    from baxter_interface import (
        Limb,
    )


# =============================================================================
# Type Definitions
# =============================================================================
_DATA_POINT = Tuple[float, float, float]
_DATA_QUARTERNION = Tuple[float, float, float]
_DATA_POSE = Tuple[_DATA_POINT, _DATA_QUARTERNION]
_PLAN = List[Dict[str, float]]


# =============================================================================
# Custom Message Objects
# =============================================================================

# ==========================================
# Baxter Interface Message - Endpoint Target
class MSG_EndpointTarget():
    '''
    Baxter Interface Message - Endpoint Target
    -
    Contains the data for the `EndpointTarget` of a particular limb.

    Attributes
    -
    - flag_cartesian : `bool`
        - READONLY.
        - Whether or not the movement mode is MODE_CARTESIAN.
    - flag_normal : `bool`
        - READONLY.
        - Whether or not the movement mode is MODE_NORMAL.
    - flag_skip : `bool`
        - READONLY.
        - Whether or not the movement mode is MODE_SKIP.
    - mode : `int`
        - Movement mode that the target must be reached using.
    - mode_str : `str`
        - READONLY.
        - String stating the type of movement mode.
    - pose : `MSG_Pose`
        - Target `Pose` of the limb.
    '''

    # ============
    # Construction
    def __init__(self, msg: EndpointTarget) -> None:
        self.mode: int = msg.mode
        self.pose = MSG_Pose(msg.pose)

    # ==========================
    # Long String Representation
    def __repr__(self) -> str:
        txt_pose = repr(self.pose).replace('\n', '\n\t')
        return (
            f'<MSG_EndpointTarget>\n\tflag_cartesian = {self.flag_cartesian}' \
            + f'\n\tflag_normal = {self.flag_normal}\n\tflag_skip = ' \
            + f'{self.flag_skip}\n\tmode = {self.mode}\n\t pose = {txt_pose}' \
            + '\n</MSG_EndpointTarget>'
        )
        
    # ===========================
    # Short String Representation
    def __str__(self) -> str:
        return (
            f'<MSG_EndpointTarget mode={self.mode_str}, pose=' \
            + f'{self.pose.get_vals(3)} />'
        )

    # =====================
    # Flag - Cartesian Mode
    @property
    def flag_cartesian(self) -> bool:
        return self.mode == EndpointTarget.MODE_CARTESIAN
    
    # ==================
    # Flag - Normal Mode
    @property
    def flag_normal(self) -> bool:
        return self.mode == EndpointTarget.MODE_NORMAL
    
    # ================
    # Flag - Skip Mode
    @property
    def flag_skip(self) -> bool:
        return self.mode == EndpointTarget.MODE_SKIP

    # ===========
    # Mode String
    @property
    def mode_str(self) -> str:
        return {
            (True, False, False): 'Cartesian',
            (False, True, False): 'Normal',
            (False, False, True): 'Skip',
        }[(self.flag_cartesian, self.flag_normal, self.flag_skip)]

# ===========================================
# Baxter Interface Message - Endpoint Targets
class MSG_EndpointTargets():
    '''
    Baxter Interface Message - Endpoint Targets
    -
    Contains the data for the `EndpointTargets` for the robot.

    Attributes
    -
    - target_l : `MSG_EndpointTarget | None`
        - Endpoint target of the left limb, if set.
    - target_r : `MSG_EndpointTarget | None`
        - Endpoint target of the right limb, if set.
    '''

    # ============
    # Construction
    def __init__(self, msg: EndpointTargets) -> None:
        self.target_l: Optional[MSG_EndpointTarget] = None
        self.target_r: Optional[MSG_EndpointTarget] = None

        if EndpointTargets.LIMB_L in msg.limbs:
            self.target_l = MSG_EndpointTarget(
                msg.targets[msg.limbs.index(EndpointTargets.LIMB_L)]
            )
        if EndpointTargets.LIMB_R in msg.limbs:
            self.target_r = MSG_EndpointTarget(
                msg.targets[msg.limbs.index(EndpointTargets.LIMB_R)]
            )

    # ==========================
    # Long String Representation
    def __repr__(self) -> str:
        txt_l = repr(self.target_l).replace('\n', '\n\t')
        txt_r = repr(self.target_r).replace('\n', '\n\t')
        return (
            f'<MSG_EndpointTargets>\n\ttarget_l = {txt_l}\n\ttarget_r = ' \
            + f'{txt_r}\n</MSG_EndpointTargets>'
        )
    
    # ===========================
    # Short String Representation
    def __str__(self) -> str:
        txt_l = 'None'
        txt_r = 'None'
        if self.target_l is not None:
            txt_l = (
                f'[{self.target_l.mode_str}, {self.target_l.pose.get_vals(3)}]'
            )
        if self.target_r is not None:
            txt_r = (
                f'[{self.target_r.mode_str}, {self.target_r.pose.get_vals(3)}]'
            )
        return f'<MSG_EndpointTargets LEFT={txt_l}, RIGHT={txt_r} />'

# ========================
# Geometry Message - Point
class MSG_Point():
    '''
    Geometry Message - Point
    -
    Contains the data for the cartesian `Point` message.

    Attributes
    -
    - x : `float`
        - X Coordinate.
    - y : `float`
        - Y Coordinate.
    - z : `float`
        - Z Coordinate.

    Methods
    -
    - get_vals(n=None) : `_DATA_POINT`
        - Instance Method.
        - Gets the X, Y, Z coordinates rounded to `n` decimal places.
    '''

    # ============
    # Construction
    def __init__(self, msg: Point) -> None:
        self.x: float = msg.x
        self.y: float = msg.y
        self.z: float = msg.z

    # =============
    # Point Message
    @property
    def msg(self) -> Point:
        ''' Point Message. '''
        return Point(
            x = self.x,
            y = self.y,
            z = self.z
        )

    # ==========================
    # Long String Representation
    def __repr__(self) -> str:
        return (
            f'<Point>\n\tx = {self.x}\n\ty = {self.y}\n\tz = {self.z}\n' \
            + '</Point>'
        )
    
    # ===========================
    # Short String Representation
    def __str__(self) -> str:
        return (
            f'<Point {self.get_vals(3)} />'
        )

    # ==========
    # Get Values
    def get_vals(self, n: Optional[int] = None) -> _DATA_POINT:
        '''
        Get Values
        -
        Gets the X, Y, X coordiantes rounded to `n` decimal places.

        Parameters
        -
        - n : `int | None`
            - Defaults to `None`, which means the coordinates will not be
                rounded. Otherwise, the coordinates will all be rounded to this
                number of decimal places.

        Returns
        -
        `_DATA_POINT`
            - Rounded X, Y, Z coordinates.
        '''

        if n is None: return (self.x, self.y, self.z,)
        return (
            round(self.x, n),
            round(self.y, n),
            round(self.z, n),
        )

# =======================
# Geometry Message - Pose
class MSG_Pose():
    '''
    Geometry Message - Pose
    -
    Contains the data for the cartesian `Pose` message.

    Attributes
    -
    - point : `MSG_Point`
        - X, Y, Z Cartesian points.
    - quarternion : `MSG_Quarternion`
        - X, Y, Z, W Quarternion points.

    Methods
    -
    - get_vals(n=None) : `_DATA_POSE`
        - Instance Method.
        - Gets the `Point` and `Quarternion` data points rounded to `n` places.
    '''

    # ============
    # Construction
    def __init__(self, msg: Pose) -> None:
        self.point = MSG_Point(msg.position)
        self.quarternion = MSG_Quarternion(msg.orientation)

    # ====
    # Pose
    @property
    def msg(self) -> Pose:
        ''' Pose Message. '''
        return Pose(
            position = self.point.msg,
            orientation = self.quarternion.msg
        )

    # ==========================
    # Long String Representation
    def __repr__(self) -> str:
        return (
            f'<MSG_Pose>\n\tpoint = {self.point}\n\tquarternion = ' \
            + f'{self.quarternion}\n</MSG_Pose>'
        )
    
    # ===========================
    # Short String Representation
    def __str__(self) -> str:
        return (
            f'<MSG_Pose {self.get_vals(3)} />'
        )

    # ==========
    # Get Values
    def get_vals(
            self, 
            n: Optional[int] = None
    ) -> _DATA_POSE:
        '''
        Get Values
        -
        Gets the `Point` and `Quarternion` data points rounded to `n` places.

        Parameters
        -
        - n : `int | None`
            - Defaults to `None`, which means the data will not be
                rounded. Otherwise, the data will all be rounded to this
                number of decimal places.

        Returns
        -
        `_DATA_QUARTERNION`
            - Rounded Point and Quarternion data.
        '''

        return (
            self.point.get_vals(n),
            self.quarternion.get_vals(n),
        )

# ==============================
# Geometry Message - Quarternion
class MSG_Quarternion():
    '''
    Geometry Message - Quarternion
    -
    Contains the data for the `Quarternion` message.

    Attributes
    -
    - x : `float`
        - X Coordinate.
    - y : `float`
        - Y Coordinate.
    - z : `float`
        - Z Coordinate.
    - w : `float`
        - W Coordinate.

    Methods
    -
    - get_vals(n=None) : `_DATA_QUARTERNION`
        - Instance Method.
        - Gets the X, Y, Z, W orientation rounded to `n` decimal places.
    '''

    # ============
    # Construction
    def __init__(self, msg: Quaternion) -> None:
        self.x: float = msg.x
        self.y: float = msg.y
        self.z: float = msg.z
        self.w: float = msg.w

    # ===================
    # Quarternion Message
    @property
    def msg(self) -> Quaternion:
        ''' Quarternion Message. '''
        return Quaternion(
            x = self.x,
            y = self.y,
            z = self.z,
            w = self.w
        )

    # ==========================
    # Long String Representation
    def __repr__(self) -> str:
        return (
            f'<Quaternion>\n\tx = {self.x}\n\ty = {self.y}\n\tz = {self.z}\n' \
            + f'\tw = {self.w}\n</Quaternion>'
        )
    
    # ===========================
    # Short String Representation
    def __str__(self) -> str:
        return (
            f'<Quaternion {self.get_vals(3)} />'
        )

    # ==========
    # Get Values
    def get_vals(
            self, 
            n: Optional[int] = None
    ) -> _DATA_QUARTERNION:
        '''
        Get Values
        -
        Gets the X, Y, Z, W orientation rounded to `n` decimal places.

        Parameters
        -
        - n : `int | None`
            - Defaults to `None`, which means the orientation will not be
                rounded. Otherwise, the orientation will all be rounded to this
                number of decimal places.

        Returns
        -
        `_DATA_QUARTERNION`
            - Rounded X, Y, Z, W orientation.
        '''

        if n is None: return (self.x, self.y, self.z, self.w,)
        return (
            round(self.x, n),
            round(self.y, n),
            round(self.z, n),
            round(self.w, n),
        )


# =============================================================================
# MoveIT Controller Object
# =============================================================================
class Controller():
    '''
    MoveIT Controller
    -
    Contains the data, publishers, and subscribers for getting ROS2 data and
    controller Baxter using MoveIT.

    Attributes
    -
    - moving : `bool`
        - Flag used for when the `Controller` is controlling the motion of the
            Baxter robot `Limb` objects.
    - moveit_group_l : `MoveGroupCommander`
        - Move group commander for the left arm of the Baxter robot.
    - moveit_group_r : `MoveGroupCommander`
        - Move group commander for the right arm of the Baxter robot.
    - limb_l : `Limb`
        - Left `Limb` of the Baxter robot.
    - limb_r : `Limb`
        - Right `Limb` of the Baxter robot.
    - targets : `MSG_EndpointTargets | None`
        - Endpoint targets to move the limbs to.

    Constants
    -
    - JOINT_TOLERANCE_BIG : `float`
        - Larger joint tolerance used to achieve a smoother motion during 
            normal control mode.
    - JOINT_TOLERANCE_SMALL : `float`
        - Small joint tolerance used for achieving higher accuracy motion
            during cartesian, skip, and final endpoint control.

    Methods
    -
    - _sub_endpoint_targets(msg) : `None`
        - Instance Method.
        - Subscriber Callback - ROS2 Endpoint Targets Receiver.
    - get_limb_pose(limb) : `Pose`
        - Class Method.
        - Gets the current pose of a particular `Limb`.
    - move() : `None`
        - Instance Method.
        - Moves the limbs of the robot to specified endpoint targets (`Pose`).
    - plan(side, goal, cartesian=False, skip_to_end=False) : `_PLAN`
        - Instance Method.
        - Creates a path plan for a single Baxter robot limb.
    '''

    # =========
    # Constants
    JOINT_TOLERANCE_BIG: float = 0.25
    JOINT_TOLERANCE_SMALL: float = 0.008726646

    # ===========
    # Constructor
    def __init__(self) -> None:
        print('Creating MoveIT ROS1 Controller')

        # initialize moveit commander
        print('| - Initializing MoveIT Commander.')
        moveit_commander.roscpp_initialize(sys.argv)

        # initialize rospy node
        print('| - Initializng ROSPY Node.')
        rospy.init_node('moveit_controller_robot')

        # moveit groups
        print('| - Creating Move Groups.')
        self.moveit_group_l = moveit_commander.MoveGroupCommander('left_arm')
        print('\t| - Left Group Done ("left_arm").')
        self.moveit_group_r = moveit_commander.MoveGroupCommander('right_arm')
        print('\t| - Right Group Done ("right_arm").')

        # robot limbs
        print('| - Initializing Robot Limbs.')
        self.limb_l = Limb('left')
        self.limb_r = Limb('right')

        print('| - Defining Local Variables')
        # moving flag
        self.moving: bool = False
        # define targets
        self.targets: Optional[MSG_EndpointTargets] = None

        # create ROS2 subscriber
        print('| - Creating Topic Subscriber (/baxter_ros2/endpoint_targets).')
        rospy.Subscriber(
            '/baxter_ros2/endpoint_targets',
            EndpointTargets,
            self._sub_endpoint_targets,
            queue_size=1,
            tcp_nodelay=True
        )

        print('MoveIT Controller Ready')

    # ====================================================
    # Subscriber Callback - ROS2 Endpoint Targets Receiver
    def _sub_endpoint_targets(self, msg: EndpointTargets) -> None:
        ''' Subscriber Callback - ROS2 Endpoint Targets Receiver. '''

        # get message data
        self.targets = MSG_EndpointTargets(msg)

        # move robot - if moving block isn't currently running
        if not self.moving: self.move()

    # =============
    # Get Limb Pose
    @classmethod
    def get_limb_pose(cls, limb: Limb) -> Pose:
        '''
        Get Limb Pose
        -
        Gets the current pose of a particular `Limb`.

        Parameters
        -
        - limb : `Limb`
            - `Limb` to get the current `Pose` from.

        Returns
        -
        `Pose`
            - `Pose` of the parsed `Limb`.
        '''

        return Pose(
            position = Point(
                x = limb._cartesian_pose['position'][0],
                y = limb._cartesian_pose['position'][1],
                z = limb._cartesian_pose['position'][2]
            ),
            orientation = Quaternion(
                x = limb._cartesian_pose['orientation'][0],
                y = limb._cartesian_pose['orientation'][1],
                z = limb._cartesian_pose['orientation'][2],
                w = limb._cartesian_pose['orientation'][3]
            )
        )
    
    # ================
    # Move Robot Limbs
    def move(self) -> None:
        '''
        Move Robot Limbs
        -
        Moves the limbs of the robot to specified endpoint targets (`Pose`).

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # set flag that movement has started
        self.moving = True

        # copy targets and clear parent targets
        if self.targets is None:
            raise RuntimeError(
                f'Controller.move() was called without any `targets` set.'
            )
        targets: MSG_EndpointTargets = self.targets
        self.targets = None

        # display movement target
        print(f'Moving: {targets}')

        # create plans
        plan_l: _PLAN = []
        plan_r: _PLAN = []
        cart_l: bool = False
        cart_r: bool = False
        if targets.target_l is not None:
            plan_l = self.plan(
                "l",
                targets.target_l.pose,
                cartesian = targets.target_l.flag_cartesian,
                skip_to_end = targets.target_l.flag_skip
            )
            cart_l = targets.target_l.flag_cartesian
        if targets.target_r is not None:
            plan_r = self.plan(
                "r",
                targets.target_r.pose,
                cartesian = targets.target_r.flag_cartesian,
                skip_to_end = targets.target_r.flag_skip
            )
            cart_r = targets.target_r.flag_cartesian

        # create movement function
        def _move(side: str) -> None:
            # get limb and plan
            limb: Limb = {'l': self.limb_l, 'r': self.limb_r}[side]
            plan: _PLAN = {'l': plan_l, 'r': plan_r}[side]
            cart: bool = {'l': cart_l, 'r': cart_r}[side]
            num_pos = len(plan)

            # move through joint positions
            for i, p in enumerate(plan):
                # if the target has changed since movement started, skip the
                #  current target
                if (
                        (self.targets is not None)
                        and (
                            (
                                (side == 'l')
                                and (self.targets.target_l is not None)
                            )
                            or (
                                (side == 'r')
                                and (self.targets.target_r is not None)
                            )
                        )
                ):
                    return None
                
                # set tolerance for movement
                tolerance: float = {
                    True: Controller.JOINT_TOLERANCE_SMALL,
                    False: Controller.JOINT_TOLERANCE_BIG,
                }[((i == num_pos-1) or (cart))]
                    
                limb.move_to_joint_positions(p, threshold=tolerance)
            return None

        # multi-thread limb movement
        threads: List[threading.Thread] = [
            threading.Thread(target=_move, args=('l',)),
            threading.Thread(target=_move, args=('r',)),
        ]
        for _t in threads: _t.start()
        for _t in threads: _t.join()

        # if new target has been set, run it
        if self.targets is not None: 
            print('New Target - Overriding Movement')
            self.move()

        # set flag that movement has ended
        self.moving = False

        return None

    # ==============
    # MoveIT Planner
    def plan(
            self,
            side: str,
            goal: MSG_Pose,
            cartesian: bool = False,
            skip_to_end: bool = False
    ) -> _PLAN:
        '''
        MoveIT Planner
        -
        Creates a path plan for a single Baxter robot limb.

        Parameters
        -
        - side : `str`
            - Side to move.
            - Valid Options:
                - `"l"` - Left.
                - `"r"` - Right.
        - goal : `MSG_Pose`
            - Goal to move the `Limb` to.
        - cartesian : `bool`
            - Flag for whether or not to make the motion in a straight line.
                Defaults to `False`, which means the straight line constraint
                will not be applied. This flag will override moveit collision
                avoidance with external objects.
        - skip_to_end : `bool`
            - Flag for whether or not to skip all of the mid-path points and
                move all of the joitns to the correct spot at once. Defaults to
                `False`, which means all of the mid-points will be applied in
                the path trajectory.

        Returns
        -
        - `_PLAN`
            - List of Points.
            - Dict containing the joint names and positions.
        '''

        # initialize variables
        joint_names: list[str]
        move_group: moveit_commander.MoveGroupCommander
        traj: RobotTrajectory
        points: list[Any]
        points_num: int

        # validate side
        if side not in ['l', 'r']:
            raise ValueError(
                f'MoveIT Path Planner Invalid: side={repr(side)}'
            )
        
        # set move group
        move_group = {'l': self.moveit_group_l, 'r': self.moveit_group_r}[side]
        limb = {'l': self.limb_l, 'r': self.limb_r}[side]

        # create plan
        if cartesian:
            (traj, _) = move_group.compute_cartesian_path(
                [self.get_limb_pose(limb), goal.msg],
                0.01,
                0
            )
        else:
            (_, traj, _, _) = move_group.plan(goal.msg)

        # create aliases
        joint_names = traj.joint_trajectory.joint_names
        points = traj.joint_trajectory.points
        points_num = len(points)

        return [
            {
                name: pt.positions[idx_n]
                for idx_n, name in enumerate(joint_names)
            }
            for idx_p, pt in enumerate(points)
            if ((not skip_to_end) or (idx_p == points_num - 1))
        ]


# =============================================================================
# Main Loop
# =============================================================================
def main():
    input()
    _ = Controller()
    rospy.spin()
    return 0
if __name__ == '__main__':
    main()


# =============================================================================
# End of File
# =============================================================================
