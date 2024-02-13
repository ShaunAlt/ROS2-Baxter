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
    Optional,
    Tuple,
    TYPE_CHECKING,
)

from contextlib import suppress

# used for math
import math

# used for multi-threading
from threading import Thread

# used for time sleeps
import time

# used for ros2 python connection
import rclpy # type: ignore
from rclpy.node import Node # type: ignore

if TYPE_CHECKING:
    # Interface Objects
    from baxter_int_ros2.baxter_int_ros2 import (
        _TIMER,
        Camera,
        df_wait,
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
        df_wait,
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

_POINT_2D = Tuple[int, int]
_POINT_3D = Tuple[int, int, int]
_POS_2D = Tuple[float, float]
_POS_3D = Tuple[float, float, float]

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
            (0.6, 0.0, 0.47,),
            (0.0, 1.0, 0.0, 0.0,)
        )
        # L = (
        #     0.6143593055481082, # E0
        #     0.9928690649588341, # E1
        #     -0.2634612003193198, # S0
        #     -0.2592427531526349, # S1
        #     -0.21399031991001521, # W0
        #     0.5756262906540015, # W1
        #     0.5871311465631421, # W0
        # )
        # R = (
        #     1.9569759901448165, # E0
        #     1.2586312364599819, # E1
        #     0.38119422578952533, # S0
        #     -0.8318010822308656, # S1
        #     -0.8870243905947405, # W0
        #     2.088898337902962, # W1
        #     1.1853836538384535, # W2
        # )

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
    COORDS_BRUSH = ( # X (Forwards/Backwards), Y (Left/Right), Z (Up/Down)
        (0.725, 0.430, -0.350), # Top Left
        (0.060, -0.640, -0.350), # Bottom Right
    )
    COORDS_PAN = ( # X (Forwards/Backwards), Y (Left/Right), Z (Up/Down)
        (0.910, 0.600, -0.350), # Top Left
        (0.235, -0.410, -0.350), # Bottom Right
        # (0.235, 0.600, -0.350), # Bottom Left
    )

    # ===========
    # Constructor
    def __init__(self) -> None:
        self.running_main = False
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

        # create instructions
        print(
            'Instructions:\n' \
            + '- To print the current positions of both limbs, press the OK' \
                + ' button on the Left Limb.\n' \
            + '- To run the main loop, press the button behind the left ' \
                + 'shoulder, then follow the console instructions.'
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
        for _t in t: 
            _t.start()
            time.sleep(0.1)
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
        if self.running_main: return
        if val: 
            self.running_main = True
            print('Running Main Program')

            # move to init
            print('| - Moving Limbs to Initialize Position')
            with suppress(): self.move_init(20)
            print('|\t| - Done')

            # calibrate grippers
            print('| - Calibrating Grippers')
            self.gripper_calibrate()
            print('|\t| - Done')

            # detach implements
            self.gripper_attach(False)

            # move to camera position
            print('| - Moving to Camera Position')
            with suppress(): self.move_camera(20)
            print('|\t| - Done - Waiting for Left Cuff Circle Press.')
            df_wait(
                lambda: self.dig_l_cuff_circle.state,
                self.dig_l_cuff_circle
            )

            # getting occupancy grid
            print('| - Getting Occupancy Grid')
            with suppress(): occ_bool, _ = self.img_r.get_occ(10)
            print(
                '|\t| - Occupancy Grid BOOL: ' \
                + self.img_r.display_occupancy_grid(bool).replace(
                    '\n',
                    '\n|\t|\t'
                )
            )
            print('|\t| - Done')

            # check point to clean
            print('| - Calculating Next Point to Clean')
            clean_point = self.get_pixel(occ_bool)
            if clean_point is None: 
                print('|\t| - No cleaning required')
                print('| - Done')
                return
            print(f'|\t| - Point to Clean: {clean_point}')
            pos_brush_org, pos_pan_org = Robot.get_pixel_pos(
                clean_point, 
                (occ_bool.shape[1], occ_bool.shape[0])
            )
            pos_brush_new, pos_pan_new = (
                (pos_brush_org[0], pos_brush_org[1]-0.2, pos_brush_org[2]),
                (pos_pan_org[0], pos_pan_org[1]+0.2, pos_pan_org[2]),
            )
            print(f'|\t| - Brush Point: {pos_brush_org} -> {pos_brush_new}')
            print(f'|\t| - Pan Point: {pos_pan_org} -> {pos_pan_new}')
            occ_bool[clean_point[1]][clean_point[0]] = 2
            print(
                '|\t| - Occupancy Grid to Clean: ' \
                + self.display_occupancy(
                    occ_bool,
                    bool
                ).replace('\n', '\n|\t|\t')
            )

            # attach gripper implements
            self.gripper_attach(True)

            # move to positions
            print('| - Moving to Sweep Starting Positions')
            with suppress():
                self._move_limbs(
                    target_l = MSG_Pose.from_coords(
                        pos_pan_new,
                        (math.sqrt(2)/2, math.sqrt(2)/2, 0.0, 0.0)
                    ),
                    target_r = MSG_Pose.from_coords(
                        pos_brush_new,
                        (0.0, 1.0, 0.0, 0.0)
                    ),
                    skip_l = True,
                    skip_r = True,
                    timeout_l = 20,
                    timeout_r = 20
                )
            print('|\t| - Done')

            # sweep
            print('| - Sweeping')
            with suppress():
                self.limb_r.set_endpoint(
                    pose = MSG_Pose.from_coords(
                        (pos_brush_new[0], pos_brush_new[1]+0.4, pos_brush_new[2]),
                        (0.0, 1.0, 0.0, 0.0)
                    ),
                    cartesian = True,
                    timeout = 10
                )
            print('|\t| - Finished Sweeping')
            self.running_main = False

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

    # ==================
    # Get Pixel Position
    @staticmethod
    def get_pixel_pos(point: _POINT_2D, size: _POINT_2D) -> Tuple[_POS_3D, _POS_3D]:
        '''
        Get Pixel Position
        -
        Gets the position of a occupancy grid pixel in Baxter's frame of
        reference.

        Parameters
        -
        - point : `_POINT_2D`
            - X and Y coordinates of the point in the occupancy grid.
        - size : `_POINT_2D`
            - Width and Height of the occupancy grid.

        Returns
        -
        `Tuple[_POS_3D, _POS_3D]`
            - `[0]`: Brush Position.
            - `[1]`: Dustpan Position.
        '''

        return (
            ( # Brush
                (
                    Robot.COORDS_BRUSH[0][0] \
                    - (
                        (point[1] / size[1]) \
                        * (Robot.COORDS_BRUSH[0][0] - Robot.COORDS_BRUSH[1][0])
                    )
                ), # X (F/B)
                (
                    Robot.COORDS_BRUSH[0][1] \
                    - (
                        (point[0] / size[0]) \
                        * (Robot.COORDS_BRUSH[0][1] - Robot.COORDS_BRUSH[1][1])
                    )
                ), # Y (L/R)
                Robot.COORDS_BRUSH[0][2], # Z (U/D)
            ),
            ( # Dustpan
                (
                    Robot.COORDS_PAN[0][0] \
                    - (
                        (point[1] / size[1]) \
                        * (Robot.COORDS_PAN[0][0] - Robot.COORDS_PAN[1][0])
                    )
                ), # X (F/B)
                (
                    Robot.COORDS_PAN[0][1] \
                    - (
                        (point[0] / size[0]) \
                        * (Robot.COORDS_PAN[0][1] - Robot.COORDS_PAN[1][1])
                    )
                ), # Y (L/R)
                Robot.COORDS_PAN[0][2], # Z (U/D)
            ),
        )

    # ==================
    # Get Pixel to Clean
    def get_pixel(self, grid: List[List[int]]) -> Optional[_POINT_2D]:
        '''
        Get Pixel to Clean
        -
        Uses the `bool` occupancy grid (1 and 0) to get the pixel to clean
        (closest to the bottom-right corner of the table), and identifies it's
        position using Baxter coordinate system.

        Parameters
        -
        - grid : `2d numpy array`
            - 2d bool occupancy grid. 1 = Occupancy. 0 = Nothing.
        
        Returns
        -
        `_POINT_2D | None`
            - Index X and Y. Top-Left = (0,0), Bottom-Right = 
                (MAX_COLS-1, MAX_ROWS-1).
            - `None` if all pixels are clean.
        '''

        def _calculate_distance(x1, y1, x2, y2) -> float:
            return ((x1-x2)**2) + ((y1-y2)**2)

        num_rows: int = grid.shape[0]
        num_cols: int = grid.shape[1]
        closest_pixel: Optional[_POINT_2D] = None
        for y, row in enumerate(grid):
            for x, cell in enumerate(row):
                if int(cell) == 1: # occupied
                    if closest_pixel is None:
                        closest_pixel = (x, y)
                    elif (
                            _calculate_distance(
                                x,
                                y, 
                                num_cols-1, 
                                num_rows-1
                            ) < _calculate_distance(
                                closest_pixel[0], 
                                closest_pixel[1], 
                                num_cols-1, 
                                num_rows-1
                            )
                    ):
                        closest_pixel = (x, y)

        return closest_pixel

    # ==================================
    # Attach / Detach Gripper Implements
    def gripper_attach(self, attach: bool = True) -> None:
        ''' Attach / Detach Gripper Implements. '''

        # move to gripper position
        print('| - Moving Limbs to Attach/Detach Position')
        with suppress(): self.move_attach(20)
        print('|\t| - Done')

        btn_txts, btn_l, btn_r, pos = {
            True: (
                ('Closing', 'Dash'),
                self.dig_l_cuff_line,
                self.dig_r_cuff_line,
                Robot.GRIPPER_CLOSED,
            ),
            False: (
                ('Opening', 'Circle'),
                self.dig_l_cuff_circle,
                self.dig_r_cuff_circle,
                Robot.GRIPPER_OPEN,
            )
        }[attach]

        for side_txt, btn, grip in ([
                ('Left', btn_l, self.grip_l),
                ('Right', btn_r, self.grip_r),
        ]):
            print(
                f'| - {btn_txts[0]} {side_txt} Gripper on {btn_txts[1]} ' \
                + 'Cuff Press'
            )
            df_wait(lambda: btn.state, btn)
            print(f'|\t| - {btn_txts[0]} {side_txt} Gripper')
            with suppress(): grip.set_pos(pos)

        print('| - Finishing on Left Cuff Circle Press')
        df_wait(
            lambda: self.dig_l_cuff_circle.state,
            self.dig_l_cuff_circle
        )
        print('|\t| - Done')

    # ==================
    # Calibrate Grippers
    def gripper_calibrate(self) -> None:
        ''' Calibrate Grippers. '''
        with suppress(): self.grip_l.configure()
        with suppress(): self.grip_l.calibrate()
        with suppress(): self.grip_r.configure()
        with suppress(): self.grip_r.calibrate()

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
    def move_camera(self, timeout=0) -> None:
        ''' Move Limbs to Camera Table Capture Position. '''
        self._move_limbs(
            POSES.CAM_TABLE.L,
            POSES.CAM_TABLE.R,
            skip_l = True,
            skip_r = True,
            timeout_l = timeout,
            timeout_r = timeout
        )
        # self.limb_l.set_positions(
        #     e0 = POSES.CAM_TABLE.L[0],
        #     e1 = POSES.CAM_TABLE.L[1],
        #     s0 = POSES.CAM_TABLE.L[2],
        #     s1 = POSES.CAM_TABLE.L[3],
        #     w0 = POSES.CAM_TABLE.L[4],
        #     w1 = POSES.CAM_TABLE.L[5],
        #     w2 = POSES.CAM_TABLE.L[6],
        # )
        # self.limb_r.set_positions(
        #     e0 = POSES.CAM_TABLE.R[0],
        #     e1 = POSES.CAM_TABLE.R[1],
        #     s0 = POSES.CAM_TABLE.R[2],
        #     s1 = POSES.CAM_TABLE.R[3],
        #     w0 = POSES.CAM_TABLE.R[4],
        #     w1 = POSES.CAM_TABLE.R[5],
        #     w2 = POSES.CAM_TABLE.R[6],
        # )

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
