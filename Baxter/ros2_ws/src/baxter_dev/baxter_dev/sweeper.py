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
        Image_Processor_V2,
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
        Image_Processor_V2,
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
        # L = MSG_Pose.from_coords(
        #     (0.6, 0.7, 0.2,),
        #     (0.0, 1.0, 0.0, 0.0,)
        # )
        # R = MSG_Pose.from_coords(
        #     (0.6, 0.0, 0.47,),
        #     (0.0, 1.0, 0.0, 0.0,)
        # )
        L = (
            0.6143593055481082, # E0
            0.9928690649588341, # E1
            -0.2634612003193198, # S0
            -0.2592427531526349, # S1
            -0.21399031991001521, # W0
            0.5756262906540015, # W1
            0.5871311465631421, # W0
        )
        R = (
            1.9569759901448165, # E0
            1.2586312364599819, # E1
            0.38119422578952533, # S0
            -0.8318010822308656, # S1
            -0.8870243905947405, # W0
            2.088898337902962, # W1
            1.1853836538384535, # W2
        )

    # ============================
    # Implement Attach/Detach Pose
    class ATTACH_DETACH():
        L = MSG_Pose.from_coords(
            (1.1, 0.1, 0.4,),
            (0.04, 0.5, 0.06, 0.8630179604,)
        )
        R = MSG_Pose.from_coords(
            (1.1, -0.1, 0.4,),
            (0.04, 0.5, 0.06, 0.8630179604,)
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

    OCCUPANCY_GRID = (40, 30) # cols, rows
    GRIPPER_OPEN = 100
    GRIPPER_CLOSED = 20

    # The following lines are copied from exact positions
    # TOP_LEFT = (0.757, 0.433, -0.332,)
    # TOP_RIGHT = (0.713, -0.646, -0.349,)
    # BOTTOM_RIGHT = (0.056, -0.640, -0.366,)
    COORDS = ( # X (Forwards/Backwards), Y (Left/Right), Z (Up/Down)
        (0.725, 0.430, -0.350), # Top Left
        (0.060, -0.640, -0.350), # Bottom Right
    )

    # ===========
    # Constructor
    def __init__(self) -> None:
        # create cameras
        self.cam_l = Camera(Topics.Camera.LEFT)
        self.cam_r = Camera(Topics.Camera.RIGHT)

        # create digital ios
        self.dig_l_cuff_circle = DigitalIO(Topics.DigitalIO.LEFT_LOWER_BUTTON)
        self.dig_l_cuff_line = DigitalIO(Topics.DigitalIO.LEFT_UPPER_BUTTON)
        self.dig_l_forearm_ok = DigitalIO(Topics.DigitalIO.LEFT_BUTTON_OK)
        self.dig_l_shoulder = DigitalIO(Topics.DigitalIO.LEFT_SHOULDER_BUTTON)
        self.dig_l_torso_ok = DigitalIO(Topics.DigitalIO.TORSO_LEFT_BUTTON_OK)
        self.dig_r_cuff_circle = DigitalIO(Topics.DigitalIO.RIGHT_LOWER_BUTTON)
        self.dig_r_cuff_line = DigitalIO(Topics.DigitalIO.RIGHT_UPPER_BUTTON)
        self.dig_r_forearm_ok = DigitalIO(Topics.DigitalIO.RIGHT_BUTTON_OK)
        self.dig_r_shoulder = DigitalIO(Topics.DigitalIO.RIGHT_SHOULDER_BUTTON)

        # create grippers
        self.grip_l = Gripper(Topics.Gripper.LEFT)
        self.grip_r = Gripper(Topics.Gripper.RIGHT)

        # create image processors
        self.img_l = Image_Processor_V2(
            Topics.Camera.LEFT,
            Robot.OCCUPANCY_GRID,
            verbose = 0
        )
        self.img_r = Image_Processor_V2(
            Topics.Camera.RIGHT,
            Robot.OCCUPANCY_GRID,
            verbose = 0
        )

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
        self.dig_l_torso_ok.state_changed.connect(
            self._state_change_l_torso_ok
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
            self.dig_l_cuff_circle,
            self.dig_l_cuff_line,
            self.dig_l_forearm_ok,
            self.dig_l_shoulder,
            self.dig_l_torso_ok,
            self.dig_r_cuff_circle,
            self.dig_r_cuff_line,
            self.dig_r_forearm_ok,
            self.dig_r_shoulder,
            self.grip_l,
            self.grip_r,
            self.img_l,
            self.img_r,
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
            print(
                'Right Limb Position + Pose:\n\t' \
                + self.limb_r.get_position_pose().replace('\n', '\n\t')
            )

    # =====================================
    # State-Change Callback - Left Shoulder
    def _state_change_l_shoulder(self, val: bool) -> None:
        ''' State-Change Callback - Left Shoulder Button. '''
        if val: 
            print('Running Main Program')

            # move to init
            print('| - Moving Limbs to Initialize Position')
            self.move_init(5)
            print('|\t| - Done')

            # detach implements
            self.gripper_attach(False)

            # move to camera position
            print('| - Moving to Camera Position')
            self.move_camera()
            print('|\t| - Done')

            # getting occupancy grid
            print('| - Getting Occupancy Grid')
            occ_bool, occ_uint8 = self.img_r.get_occ(5)
            print(
                '|\t| - Occupancy Grid BOOL: ' \
                + self.display_occupancy(
                    occ_bool, 
                    bool
                ).replace('\n', '\n|\t|\t')
            )
            print('|\t| - Done')

            # attach gripper implements
            self.gripper_attach(True)

    # =====================================
    # State-Change Callback - Left Torso OK
    def _state_change_l_torso_ok(self, val: bool) -> None:
        ''' State-Change Callback - Left Torso OK Button. '''
        if val:
            print('Getting Table Occupancy Grids.')
            print('| - Moving to Position.')
            self.move_camera()
            print('| - Getting Occupancy Grids.')
            occ_bool, occ_uint8 = self.img_r.get_occ() # numpy 2d arrays
            print(
                '| - UINT8 Occupancy Grid: ' \
                + self.display_occupancy(occ_uint8, int).replace('\n', '\n|    ')
            )
            print(
                '| - BOOL Occupancy Grid: ' \
                + self.display_occupancy(occ_bool, bool).replace('\n', '\n|    ')
            )
            print('Done Getting Occupancy Grids.')

    # ========================================
    # State-Change Callback - Right Forearm OK
    def _state_change_r_forearm_ok(self, val: bool) -> None:
        ''' State-Change Callback - Right Forearm OK Button. '''
        if val:
            print('Moving Limbs to Attach/Detach Position.')
            self.move_attach()
            print('Done moving limbs to ATTACH_DETACH.')

    # ======================================
    # State-Change Callback - Right Shoulder
    def _state_change_r_shoulder(self, val: bool) -> None:
        ''' State-Change Callback - Right Shoulder Button. '''
        if val: 
            print('Moving Limbs to Camera Position')
            self.move_camera()
            print('Done moving limbs to CAM_TABLE.')

    # ======================
    # Display Occupancy Grid
    def display_occupancy(self, grid: List[List[Any]], _type: Any) -> str:
        '''
        Display Occupancy Grid
        -
        Displays an occupancy grid in a readable format.

        Parameters
        -
        - grid : `List[List[Any]] | numpy 2D array`
            - 2D array/list to be printed out.
        - _type : `Type[int] | Type[bool]`
            - `int` or `bool` type object indicating the type of object.

        Returns
        -
        `str`
            - String containing pretty-printed occupancy grid.
        '''

        output: str = '[\n'
        for row in grid:
            output += '\t['
            if _type is int:
                output += ', '.join([f'{cell:03}' for cell in row])
            elif _type is bool:
                output += ', '.join([str(int(cell)) for cell in row])
            output += '],\n'
        output += ']'

        return output

    # ==================================
    # Attach / Detach Gripper Implements
    def gripper_attach(self, attach: bool = True) -> None:
        ''' Attach / Detach Gripper Implements. '''

        # move to gripper position
        print('| - Moving Limbs to Attach/Detach Position')
        self.move_attach(5)
        print('|\t| - Done')

        # open grippers
        print('| - Opening Left Gripper on Circle Cuff Press')
        while not self.dig_l_cuff_circle.state: pass
        print('|\t| - Opening Left Gripper')
        self.grip_l.set_pos(Robot.GRIPPER_OPEN)
        print('| - Opening Right Gripper on Circle Cuff Press')
        while not self.dig_r_cuff_circle.state: pass
        print('|\t| - Opening Right Gripper')
        self.grip_r.set_pos(Robot.GRIPPER_OPEN)

        # attach
        if attach:
            print('| - Closing Left Gripper on Dash Cuff Press')
            while not self.dig_l_cuff_line.state: pass
            print('|\t| - Closing Left Gripper')
            self.grip_l.set_pos(Robot.GRIPPER_CLOSED)
            print('| - Closing Right Gripper on Dash Cuff Press')
            while not self.dig_r_cuff_line.state: pass
            self.grip_r.set_pos(Robot.GRIPPER_CLOSED)

        print('| - Finishing on Left Cuff Circle Press')
        while not self.dig_l_cuff_circle.state: pass
        print('|\t| - Done')


    # ====================================
    # Move Limbs to Attach/Detach Position
    def move_attach(self, timeout=0) -> None:
        ''' Move Limbs to Attach/Detach Position. '''
        self._move_limbs(
            POSES.ATTACH_DETACH.L,
            POSES.ATTACH_DETACH.R,
            skip_l = True,
            skip_r = True,
            timeout_l=timeout,
            timeout_r=timeout
        )
            
    # ===========================================
    # Move Limbs to Camera Table Capture Position
    def move_camera(self) -> None:
        ''' Move Limbs to Camera Table Capture Position. '''
        # self._move_limbs(
        #     POSES.CAM_TABLE.L,
        #     POSES.CAM_TABLE.R,
        #     skip_l = True,
        #     skip_r = True,
        # )
        self.limb_l.set_positions(
            e0 = POSES.CAM_TABLE.L[0],
            e1 = POSES.CAM_TABLE.L[1],
            s0 = POSES.CAM_TABLE.L[2],
            s1 = POSES.CAM_TABLE.L[3],
            w0 = POSES.CAM_TABLE.L[4],
            w1 = POSES.CAM_TABLE.L[5],
            w2 = POSES.CAM_TABLE.L[6],
        )
        self.limb_r.set_positions(
            e0 = POSES.CAM_TABLE.R[0],
            e1 = POSES.CAM_TABLE.R[1],
            s0 = POSES.CAM_TABLE.R[2],
            s1 = POSES.CAM_TABLE.R[3],
            w0 = POSES.CAM_TABLE.R[4],
            w1 = POSES.CAM_TABLE.R[5],
            w2 = POSES.CAM_TABLE.R[6],
        )

    # =================================
    # Move Limbs to Initialize Position
    def move_init(self, timeout=0) -> None:
        ''' Move Limbs to Initialize Position. '''
        self._move_limbs(
            POSES.INIT.L,
            POSES.INIT.R,
            skip_l = True,
            skip_r = True,
            timeout_l=timeout,
            timeout_r=timeout
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
