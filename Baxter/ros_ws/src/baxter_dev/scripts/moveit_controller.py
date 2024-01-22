#!/usr/bin/env python3

#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter MoveIT ROS1 Controller
-
ROS1 subscriber and publisher controller which reads the planned paths defined
by MoveIT and publishes them to Baxter.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# used for type hinting
from typing import (
    Any,
    Callable,
    Dict,
    List,
    Optional,
    Tuple,
    TYPE_CHECKING,
    Union,
)

# used for threading
import threading

# used for date/time
from datetime import datetime

# ROS1-Python Package
import rospy # type: ignore

# MoveIT Message
from moveit_msgs.msg import ( # type: ignore
    DisplayTrajectory,
    RobotTrajectory,
)

# Geometry Message
from geometry_msgs.msg import ( # type: ignore
    Point,
    Pose,
    Quaternion,
)

# MoveIT Commander
import moveit_commander # type: ignore

# used for system
import sys

if TYPE_CHECKING:
    # Baxter Interface Limb Object
    from baxter_legacy.baxter_interface.src.baxter_interface import (
        Limb,
        DigitalIO,
    )
else:
    # Baxter Interface Objects
    from baxter_interface import (
        Limb,
        DigitalIO,
    )


# =============================================================================
# Joint Trajectory Object Definitions
# =============================================================================
class Header():
    ''' std_msgs.Header Message. '''
    def __init__(
            self,
            msg: Any
    ) -> None:
        self.seq: int = msg.seq
        self.stamp: datetime = msg.stamp
        self.frame_id: str = msg.frame_id
    def __repr__(self) -> str:
        return (
            f'<Header>\n\tseq = {self.seq}\n\tstamp = {self.stamp}' \
                + f'\n\tframe_id = {self.frame_id}\n</Header>'
        )
    def __str__(self) -> str:
        return (
            f'<Header seq={self.seq}, stamp={self.stamp}, frame_id=' \
                + f'{self.frame_id} />'
        )

class JointTrajectory():
    ''' 
    Joint Trajectory Object Definition 
    -
    Contains all of the information required for a trajectory of joint
    positions for the Baxter robot.

    Attributes
    -
    - model_id : `str`
        - Model ID for the path being generated. Should be "baxter", if not
            then this entire message should be ignored.
    - header : `Header`
        - No current use.
    - joint_cmds : `list[dict[str, _JOINT_POINT]]`
        - Joint names, as well as the commanded position, velocity, and
            acceleration. Each dict in the list is a position along the path.
    '''

    # =====
    # Types
    _JOINT_POINT = Tuple[float, float, float] # pos, vel, acc

    # ===========
    # Constructor
    def __init__(self, msg: DisplayTrajectory) -> None:
        '''
        JointTrajectory Constructor
        -
        Creates a `JointTrajectory` object.
        
        Parameters
        -
        - msg: `DisplayTrajectory`
            - `DisplayTrajectory` message.
            
        Returns
        -
        None
        '''

        # validate model_id
        if msg.model_id != 'baxter':
            raise ValueError(
                'Tried to create JointTrajectory with model_id = ' \
                    + f'{repr(msg.model_id)}'
            )

        # validate trajectory
        if len(msg.trajectory) != 1:
            raise ValueError(
                'Tried to create JointTrajectory with len(trajectory) = ' \
                    + f'{len(msg.trajectory)}'
            )
        _jt: Any = msg.trajectory[0].joint_trajectory

        # create model_id and header
        self.model_id: str = msg.model_id
        self.header = Header(_jt.header)

        # create joint trajectories
        self.joint_cmds: List[Dict[str, JointTrajectory._JOINT_POINT]] = []
        for i in range(len(_jt.points)):
            _joint_cmd: Dict[str, JointTrajectory._JOINT_POINT] = {}
            for j in range(len(_jt.joint_names)):
                _joint_cmd[_jt.joint_names[j]] = (
                    _jt.points[i].positions[j],
                    _jt.points[i].velocities[j],
                    _jt.points[i].accelerations[j],
                )
            self.joint_cmds.append(_joint_cmd)

    # ===================
    # Get Limb Trajectory
    def get_limb_trajectory(self, side: str) -> List[Dict[str, float]]:
        ''' Get Limb Trajectory. side = {'l', 'r'}. '''

        # left limbs
        if side == 'l':
            return [
                {
                    _joint_name: _joint_vals[0]
                    for _joint_name, _joint_vals in _point.items()
                    if _joint_name.startswith('left')
                }
                for _point in self.joint_cmds
            ]

        # right limbs
        elif side == 'r':
            return [
                {
                    _joint_name: _joint_vals[0]
                    for _joint_name, _joint_vals in _point.items()
                    if _joint_name.startswith('right')
                }
                for _point in self.joint_cmds
            ]

        # invalid
        raise ValueError(f'Get Limb Trajectory: side = {repr(side)}')

# =============================================================================
# Robot Definition
# =============================================================================
class Robot():
    ''' Baxter Robot Definition. '''

    JOINT_TOLERANCE_BIG: float = 0.25 # 0.2
    JOINT_TOLERANCE_SMALL: float = 0.008726646

    # ===========
    # Constructor
    def __init__(self):
        ''' Baxter Robot Definition Constructor. '''

        # initialize moveit commander
        moveit_commander.roscpp_initialize(sys.argv)

        # initialize rospy node
        rospy.init_node('moveit_controller_robot', anonymous = True)

        # moveit objects
        # self.moveit_robot = moveit_commander.RobotCommander()
        # self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.moveit_group_l = moveit_commander.MoveGroupCommander('left_arm')
        self.moveit_group_r = moveit_commander.MoveGroupCommander('right_arm')
        self.moveit_path_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size = 20
        )
        # self.moveit_planning_frame = self.moveit_group.get_planning_frame()
        # self.moveit_eef_link = self.moveit_group.get_end_effector_link()
        # self.moveit_group_names = self.moveit_robot.get_group_names()

        # create robot limbs
        self.limb_l = Limb('left')
        self.limb_r = Limb('right')
        self._subscriber_moveit_planner = rospy.Subscriber(
            '/move_group/display_planned_path',
            DisplayTrajectory,
            self._sub_moveit_planner,
            queue_size=1,
            tcp_nodelay=True
        )

        # DigitalIO buttons
        # left buttons
        self.btn_left_back = DigitalIO('left_button_back')
        self.btn_left_back.state_changed.connect(
            self._move_to_pose_both_cartesian
        )
        self.btn_left_circle = DigitalIO('left_lower_button')
        self.btn_left_circle.state_changed.connect(self._save_pose_left)
        self.btn_left_dash = DigitalIO('left_upper_button')
        self.btn_left_dash.state_changed.connect(self._move_to_pose_left)
        self.btn_left_ok = DigitalIO('left_button_ok')
        self.btn_left_ok.state_changed.connect(self._move_to_pose_both_normal)
        self.btn_left_show = DigitalIO('left_button_show')
        self.btn_left_show.state_changed.connect(self._move_to_pose_both_skip)
        # right buttons
        self.btn_right_back = DigitalIO('right_button_back')
        self.btn_right_back.state_changed.connect(
            self._move_to_pose_both_cartesian
        )
        self.btn_right_circle = DigitalIO('right_lower_button')
        self.btn_right_circle.state_changed.connect(self._save_pose_right)
        self.btn_right_dash = DigitalIO('right_upper_button')
        self.btn_right_dash.state_changed.connect(self._move_to_pose_right)
        self.btn_right_ok = DigitalIO('right_button_ok')
        self.btn_right_ok.state_changed.connect(self._move_to_pose_both_normal)
        self.btn_right_show = DigitalIO('right_button_show')
        self.btn_right_show.state_changed.connect(self._move_to_pose_both_skip)

        # saved poses
        self.pose_left: Optional[Pose] = None
        self.pose_right: Optional[Pose] = None

        # print description
        print(
            'Initialization Completed: MoveIT Controllers Ready.\n\t' \
                + '- Press the Circle button on the cuff of a limb to save' \
                + ' the Pose of that limb.\n\t- Press the Dash button on' \
                + ' the cuff of a limb to move only that arm to the saved ' \
                + 'Pose. \n\t- Press the Back button on either forearm to ' \
                + 'move both arms Cartesian to their saved Poses. \n\t- ' \
                + 'Press the OK button on either forearm to move both arms ' \
                + 'to their saved Poses normally. \n\t- Press the Show ' \
                + 'button on either forearm to move both arms to their saved' \
                + ' Poses skipping the path planning and moving to the final' \
                + ' Pose immediately.'
        )

    # =============================================
    # Move to Pose Callback - Both - Cartesian Path
    def _move_to_pose_both_cartesian(self, val: bool) -> None:
        ''' Move to Pose Callback - Both Arms - Cartesian Path. '''
        if val:
            self.move_limbs(
                goal_l = self.pose_left,
                goal_r = self.pose_right,
                cartesian = True
            )

    # ==========================================
    # Move to Pose Callback - Both - Normal Path
    def _move_to_pose_both_normal(self, val: bool) -> None:
        ''' Move to Pose Callback - Both Arms - Normal Path. '''
        if val:
            self.move_limbs(
                goal_l = self.pose_left,
                goal_r = self.pose_right
            )

    # ========================================
    # Move to Pose Callback - Both - Skip Path
    def _move_to_pose_both_skip(self, val: bool) -> None:
        ''' Move to Pose Callback - Both Arms - Skip Path. '''
        if val:
            self.move_limbs(
                goal_l = self.pose_left,
                goal_r = self.pose_right,
                skip_to_end = True
            )

    # ============================
    # Move to Pose Callback - Left
    def _move_to_pose_left(self, val: bool) -> None:
        ''' Move to Pose Callback - Left Limb. '''
        if val: 
            self.move_limbs(
                goal_l = self.pose_left
            )

    # =============================
    # Move to Pose Callback - Right
    def _move_to_pose_right(self, val: bool) -> None:
        ''' Move to Pose Callback - Right Limb. '''
        if val: 
            self.move_limbs(
                goal_r = self.pose_right
            )
        
    # ==============
    # MoveIT Planner
    def _plan(
            self,
            side: str,
            goal: Pose,
            cartesian: bool = False,
            skip_to_end: bool = False
    ) -> List[Dict[str, float]]:
        '''
        MoveIT Planner
        -
        Creates a path plan for a single robot limb.

        Parameters
        -
        - side : `str`
            - Side to move.
            - Valid Options:
                - "l" - Left
                - "r" - Right
        - goal : `Pose`
            - Goal to move the Limb to.
        - cartesian : `bool`
            - Flag for whether or not to make the motion in a straight line.
                Defaults to `False`, which means the straight line constraint
                will not be applied. This flag will override moveit collision
                avoidance with external objects.
        - skip_to_end : `bool`
            - Flag for whether or not to skip all of the mid-path points and
                move all of the joints to the correct spot at once. Defaults to
                `False`, which means all of the mid-points will be applied in
                the path trajectory.

        Returns
        -
        - list[dict[str, float]]
            - List of Points
                - Dict containing the joint name and joint position.
        '''

        # initialize variables
        joint_names: list[str]
        move_group: moveit_commander.MoveGroupCommander
        plan: RobotTrajectory
        points: list[Any]
        points_num: int

        # validate side
        if side not in ['l', 'r']:
            raise ValueError(
                f'MoveIT Path Planner Invalid: side={repr(side)}'
            )
        
        # set move group
        move_group = {'l': self.moveit_group_l, 'r': self.moveit_group_r}[side]

        # create plan
        if cartesian:
            (plan, _) = move_group.compute_cartesian_path(
                [move_group.get_current_pose().pose, goal],
                0.01,
                0
            )
        else:
            (_, plan, _, _) = move_group.plan(goal)

        # create aliases
        joint_names = plan.joint_trajectory.joint_names
        points = plan.joint_trajectory.points
        points_num = len(points)

        return [
            {
                name: pt.positions[idx_n]
                for idx_n, name in enumerate(joint_names)
            }
            for idx_p, pt in enumerate(points)
            if ((not skip_to_end) or (idx_p == points_num - 1))
        ]
    
    # =========
    # Save Pose
    def _save_pose(self, side: str) -> None:
        '''
        Save Pose
        -
        Saves the `Pose` of the given `Limb`.

        Parameters
        -
        - side : `str`
            - Which `Limb` to save.
            - Valid Options:
                - `"left"` - Left.
                - `"right"` - Right.

        Returns
        -
        None
        '''

        if side not in ['left', 'right']:
            raise ValueError(
                f'Tried to Save Pose with invalid side={repr(side)}'
            )
        
        _limb = {'left': self.limb_l, 'right': self.limb_r}[side]
        _endpoint_data = _limb._cartesian_pose
        _pose = Pose(
            position = Point(
                x = _endpoint_data['position'][0],
                y = _endpoint_data['position'][1],
                z = _endpoint_data['position'][2]
            ),
            orientation = Quaternion(
                x = _endpoint_data['orientation'][0],
                y = _endpoint_data['orientation'][1],
                z = _endpoint_data['orientation'][2],
                w = _endpoint_data['orientation'][3]
            )
        )

        if side == 'left':
            self.pose_left = _pose
        else:
            self.pose_right = _pose

        print(f'Saved {side} Pose: {_pose}')

    # =========================
    # Save Pose Callback - Left
    def _save_pose_left(self, val: bool) -> None:
        ''' Save Pose Callback - Left Limb. '''
        if val: self._save_pose('left')

    # ==========================
    # Save Pose Callback - Right
    def _save_pose_right(self, val: bool) -> None:
        ''' Save Pose Callback - Right Limb. '''
        if val: self._save_pose('right')

    # =========================================
    # Subscriber Callback - MoveIT Path Planner
    def _sub_moveit_planner(self, msg: DisplayTrajectory) -> None:
        ''' Subscriber Callback - MoveIT Path Planner. '''

        # create joint trajectory
        trajectory = JointTrajectory(msg)
        print(f'Read Trajectory: {len(trajectory.joint_cmds)} Points')

        # # go to each of the required positions
        def move_limb(side: str):
            ''' Move Limb. side = {l, r}. '''
            pts: List[Dict[str, float]] = trajectory.get_limb_trajectory(side)
            limb: Limb = {'l': self.limb_l, 'r': self.limb_r}[side]
            for _p in pts[:-1]:
                limb.move_to_joint_positions(
                    _p,
                    threshold = Robot.JOINT_TOLERANCE_BIG
                )
            for _p in pts[-1:]:
                limb.move_to_joint_positions(
                    _p,
                    threshold = Robot.JOINT_TOLERANCE_SMALL
                )
            print(f'Finished Moving Limb {side}')

        print('Moving Left Arm.')
        threading.Thread(target=move_limb, args=('l', )).start()
        print('Moving Right Arm.')
        threading.Thread(target=move_limb, args=('r', )).start()

    # ================
    # Move Robot Limbs
    def move_limbs(
            self,
            goal_l: Optional[Pose] = None,
            goal_r: Optional[Pose] = None,
            cartesian: bool = False,
            skip_to_end: bool = False
    ) -> None:
        '''
        Move Robot Limbs
        -
        Moves the limbs of the robot to specified poses.

        Parameters
        -
        - goal_l : `Pose | None`
            - `Pose` to set the left `Limb` to. Defaults to `None`, which means
                the `Limb` goal will be set as its current position.
        - goal_r : `Pose | None`
            - `Pose` to set the right `Limb` to. Defaults to `None`, which
                means the `Limb` goal will be set as its current position.
        - cartesian : `bool`
            - Flag for whether or not to make the motion in a straight line.
                Defaults to `False`, which means the straight line constraint
                will not be applied. This flag will override moveit collision
                avoidance with external objects.
        - skip_to_end : `bool`
            - Flag for whether or not to skip all of the mid-path points and
                move all of the joints to the correct spot at once. Defaults to
                `False`, which means all of the mid-points will be applied in
                the path trajectory.

        Returns
        -
        None
        '''

        # create left path plan
        def move(side: str) -> None:
            ''' Move Limb. side = {l, r}. '''
            print(f'Moving Limb {side}')

            # get goal
            goal: Optional[Pose] = {'l': goal_l, 'r': goal_r}[side]
            if goal is None: return None
                # goal = {
                #     'l': self.moveit_group_l,
                #     'r': self.moveit_group_r,
                # }[side].get_current_pose().pose

            # create limb + plan
            limb: Limb = {'l': self.limb_l, 'r': self.limb_r}[side]
            plan: List[Dict[str, float]] = self._plan(
                side,
                goal,
                cartesian = cartesian,
                skip_to_end = skip_to_end
            )

            # move joints
            for _p in plan[:-1]:
                limb.move_to_joint_positions(
                    _p,
                    threshold = Robot.JOINT_TOLERANCE_BIG
                )
            for _p in plan[-1:]:
                limb.move_to_joint_positions(
                    _p,
                    threshold = Robot.JOINT_TOLERANCE_SMALL
                )
            print(f'Finished Moving Limb {side}')

        threading.Thread(target=move, args=('l',)).start()
        threading.Thread(target=move, args=('r',)).start()

    # ===============================
    # Publisher - MoveIT Path Planner
    def pub_moveit_planner_old(
            self,
            side: str,
            goal: Pose,
            cartesian: bool = False,
            skip_to_end: bool = False
    ) -> None:
        '''
        Publisher - MoveIT Path Planner
        -
        Publishes a moveit path plan to the required topic based on the parsed
        goals and flags.

        Parameters
        -
        - side : `str`
            - Side to move.
            - Valid Options:
                - "l" - Left
                - "r" - Right
        - goal : `Pose`
            - Goal to move the Limb to.
        - cartesian : `bool`
            - Flag for whether or not to make the motion in a straight line.
                Defaults to `False`, which means the straight line constraint
                will not be applied. This flag will override moveit collision
                avoidance with external objects.
        - skip_to_end : `bool`
            - Flag for whether or not to skip all of the mid-path points and
                move all of the joints to the correct spot at once. Defaults to
                `False`, which means all of the mid-points will be applied in
                the path trajectory.
            - Not currently in use.

        Returns
        -
        None
        '''

        # validate side
        if side not in ['l', 'r']:
            raise ValueError(
                f'Publisher - MoveIT Path Planner Invalid: side={repr(side)}'
            )
        
        # set move group
        move_group = {'l': self.moveit_group_l, 'r': self.moveit_group_r}[side]

        # create plan
        if cartesian:
            (plan, _) = move_group.compute_cartesian_path(
                [move_group.get_current_pose().pose, goal],
                0.01,
                0
            )
        else:
            (_, plan, _, _) = move_group.plan(goal)

        # publish trajectory
        trajectory = DisplayTrajectory()
        trajectory.trajectory_start = self.moveit_robot.get_current_state()
        trajectory.trajectory.append(plan)
        self.moveit_path_publisher.publish(trajectory)


# =============================================================================
# Main Loop
# =============================================================================
def main():
    bot = Robot()
    rospy.spin()
    print('Done')
    return 0

if __name__ == '__main__':
    main()


# =============================================================================
# End of File
# =============================================================================
