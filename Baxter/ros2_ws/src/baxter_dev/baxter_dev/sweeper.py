#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter Robot Sweeping Script
-
Contains the robot definition and designed functionality to get the Baxter
robot to sweep up objects on a pre-set table.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# used for running the main function
import sys
# used for type hinting
from typing import (
    Any,
    List,
    TYPE_CHECKING,
)

# used for multi-threading
from threading import Thread

# used for ros2 python connection
import rclpy # type: ignore
from rclpy.node import Node # type: ignore

if TYPE_CHECKING:
    # Interface Objects
    from baxter_int_ros2.baxter_int_ros2 import (
        _TIMER,
        Camera,
        DigitalIO,
        Gripper,
        Image_Processor,
        Limb,
        MSG_Pose,
        ROS2_Node,
        Topics,
    )
else:
    # Interface Objects
    from baxter_int_ros2 import (
        _TIMER,
        Camera,
        DigitalIO,
        Gripper,
        Image_Processor,
        Limb,
        MSG_Pose,
        ROS2_Node,
        Topics,
    )

#!TODO: For Sweeping the Table, Keep Quarternion Angles:
'''
X, Z = 0
Y = 1 <- Straight Ahead
Slowly change W and Y (so the square sum is 1) to make the angle left and
    right.
Y will always be positive, W between -1 and 1.
'''


# =============================================================================
# Preset Poses
# =============================================================================
class POSES():
    ''' Preset Poses. '''

    # ===================
    # Initialization Pose
    class INIT():
        L = MSG_Pose.from_coords(
            (0.6, 0.2, 0.1,),
            (0.0, 1.0, 0.0, 0.0,)
        )
        R = MSG_Pose.from_coords(
            (0.6, -0.2, 0.1,),
            (0.0, 1.0, 0.0, 0.0,)
        )

    # ======================
    # Camera View Table Pose
    class CAM_TABLE():
        L = MSG_Pose.from_coords(
            (0.6, 0.7, 0.2,),
            (0.0, 1.0, 0.0, 0.0,)
        )
        R = MSG_Pose.from_coords(
            (0.55, 0.0, 0.5,),
            (0.0, 1.0, 0.0, 0.0,)
        )

    # ============================
    # Implement Attach/Detach Pose
    class ATTACH_DETACH():
        L = MSG_Pose.from_coords(
            (0.0, 0.0, 0.0,),
            (0.0, 0.0, 0.0, 0.0,)
        )
        R = MSG_Pose.from_coords(
            (0.0, 0.0, 0.0,),
            (0.0, 0.0, 0.0, 0.0,)
        )


# =============================================================================
# Robot Definition
# =============================================================================
class Robot():
    '''
    Baxter Robot
    -
    Contains all of the functions and object definitions required for creating
    the sweeping functionality.
    '''

    # ===========
    # Constructor
    def __init__(self) -> None:
        # create cameras
        self.cam_l = Camera(Topics.Camera.LEFT)
        self.cam_r = Camera(Topics.Camera.RIGHT)

        # create digital ios
        self.dig_l_forearm_ok = DigitalIO(Topics.DigitalIO.LEFT_BUTTON_OK)
        self.dig_l_shoulder = DigitalIO(Topics.DigitalIO.LEFT_SHOULDER_BUTTON)
        self.dig_r_forearm_ok = DigitalIO(Topics.DigitalIO.RIGHT_BUTTON_OK)
        self.dig_r_shoulder = DigitalIO(Topics.DigitalIO.RIGHT_SHOULDER_BUTTON)

        # create limbs
        self.limb_l = Limb(Topics.Limb.LEFT)
        self.limb_r = Limb(Topics.Limb.RIGHT)

        # digital io state-change methods
        self.dig_l_forearm_ok.state_changed.connect(
            self._state_change_l_forearm_ok
        )
        self.dig_l_shoulder.state_changed.connect(
            self._state_change_l_shoulder
        )
        self.dig_r_forearm_ok.state_changed.connect(
            self._state_change_r_forearm_ok
        )
        self.dig_r_shoulder.state_changed.connect(
            self._state_change_r_shoulder
        )

        return None

    # ==========
    # Nodes List
    @property
    def nodes(self) -> List[ROS2_Node]:
        return [
            self.cam_l,
            self.cam_r,
            self.dig_l_forearm_ok,
            self.dig_l_shoulder,
            self.dig_r_forearm_ok,
            self.dig_r_shoulder,
            self.limb_l,
            self.limb_r,
        ]

    # ================================
    # Move Limbs to Specific Positions
    def _move_limbs(
            self,
            target_l: MSG_Pose,
            target_r: MSG_Pose,
            cartesian_l: bool = False,
            skip_l: bool = False,
            cartesian_r: bool = False,
            skip_r: bool = False,
            timeout_l: int = 0,
            timeout_r: int = 0
    ) -> None:
        ''' Move Limbs to Specific Positions. '''

        t = [
            Thread(
                target=self.limb_l.set_endpoint, 
                args=(
                    target_l, 
                    cartesian_l,
                    skip_l,
                    timeout_l,
                )
            ),
            Thread(
                target=self.limb_r.set_endpoint,
                args=(
                    target_r,
                    cartesian_r,
                    skip_r,
                    timeout_r
                )
            )
        ]
        for _t in t: _t.start()
        for _t in t: _t.join()

    # =======================================
    # State-Change Callback - Left Forearm OK
    def _state_change_l_forearm_ok(self, val: bool) -> None:
        ''' State-Change Callback - Left Forearm OK Button. '''
        if val:
            print(
                'Left Limb Position + Pose:\n\t' \
                + self.limb_l.get_position_pose().replace('\n', '\n\t')
            )

    # =====================================
    # State-Change Callback - Left Shoulder
    def _state_change_l_shoulder(self, val: bool) -> None:
        ''' State-Change Callback - Left Shoulder Button. '''
        if val: 
            print('Moving Limbs to Initialize Position')
            self.move_init()
            print('Done moving limbs to INIT.')

    # ========================================
    # State-Change Callback - Right Forearm OK
    def _state_change_r_forearm_ok(self, val: bool) -> None:
        ''' State-Change Callback - Right Forearm OK Button. '''
        if val:
            print(
                'Right Limb Position + Pose:\n\t' \
                + self.limb_r.get_position_pose().replace('\n', '\n\t')
            )

    # ======================================
    # State-Change Callback - Right Shoulder
    def _state_change_r_shoulder(self, val: bool) -> None:
        ''' State-Change Callback - Right Shoulder Button. '''
        if val: 
            print('Moving Limbs to Camera Position')
            self.move_camera()
            print('Done moving limbs to CAM_TABLE.')
            
    # ====================================
    # Move Limbs to Attach/Detach Position
    def move_attach(self) -> None:
        ''' Move Limbs to Attach/Detach Position. '''
        self._move_limbs(
            POSES.ATTACH_DETACH.L,
            POSES.ATTACH_DETACH.R,
            skip_l = True,
            skip_r = True,
            timeout_l = 10,
            timeout_r = 10
        )
            
    # ===========================================
    # Move Limbs to Camera Table Capture Position
    def move_camera(self) -> None:
        ''' Move Limbs to Camera Table Capture Position. '''
        self._move_limbs(
            POSES.CAM_TABLE.L,
            POSES.CAM_TABLE.R,
            skip_l = True,
            skip_r = True,
            timeout_l = 10,
            timeout_r = 10
        )

    # =================================
    # Move Limbs to Initialize Position
    def move_init(self) -> None:
        ''' Move Limbs to Initialize Position. '''
        self._move_limbs(
            POSES.INIT.L,
            POSES.INIT.R,
            skip_l = True,
            skip_r = True,
            timeout_l = 10,
            timeout_r = 10
        )


# =============================================================================
# Main Function
# =============================================================================
def main(args: None = None):
    print('Running Baxter Sweeper')
    print(
        'To Stream Camera Data: Run the `streamer` with the following' \
            + 'topics:\n' \
        + '| - Left Hand Raw Camera Data: left_hand_camera/image_data\n' \
        + '| - Right Hand Raw Camera Data: right_hand_camera/image_data\n' \
    )
    rclpy.init()
    print('Creating Robot Nodes')
    r = Robot()
    _exec = rclpy.executors.MultiThreadedExecutor()
    for node in r.nodes:
        _exec.add_node(node)
    _exec.spin()
    for node in r.nodes:
        node.destroy_n()

if __name__ == '__main__':
    sys.exit(main())


# =============================================================================
# End of File
# =============================================================================
