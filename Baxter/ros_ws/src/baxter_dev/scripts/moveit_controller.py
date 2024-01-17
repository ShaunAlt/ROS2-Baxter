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
from moveit_msgs.msg import DisplayTrajectory # type: ignore

if TYPE_CHECKING:
    # Baxter Interface Limb Object
    from baxter_legacy.baxter_interface.src.baxter_interface import Limb
else:
    # Baxter Interface Objects
    from baxter_interface import Limb


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
        self.limb_l = Limb('left')
        self.limb_r = Limb('right')
        self._subscriber_moveit_planner = rospy.Subscriber(
            '/move_group/display_planned_path',
            DisplayTrajectory,
            self._sub_moveit_planner,
            queue_size=1,
            tcp_nodelay=True
        )
        
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
        


# =============================================================================
# Main Loop
# =============================================================================
def main():
    rospy.init_node('baxter_moveit_controller', anonymous = True)
    bot = Robot()
    rospy.spin()
    print('Done')
    return 0

if __name__ == '__main__':
    main()


# =============================================================================
# End of File
# =============================================================================
