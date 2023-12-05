#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter Control Module
-
Contains custom-defined definitions of the Baxter robot interface objects,
with more documentation and type-hinting to help assist design and development.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# Baxter Interface Objects
from baxter_interface import (
    AnalogIO,
    CameraController,
    DigitalIO,
    Gripper,
    Head,
    Limb,
    Navigator,
    RobotEnable,
    RobustController,
    JOINT_ANGLE_TOLERANCE,
    HEAD_PAN_ANGLE_TOLERANCE,
    SDK_VERSION,
    CHECK_VERSION,
    VERSIONS_SDK2ROBOT,
    VERSIONS_SDK2GRIPPER,
)

# ROS-Python Package
import rospy

# Type-Hinting
from typing import (
    Any,
    Dict,
    List,
)


# =============================================================================
# Robot Objects
# =============================================================================
class Button():
    '''
    Robot Button
    -
    Base object which is used for providing the generic functionality for all
    of the buttons in the Baxter robot.

    Attributes
    -
    - _btn : `DigitalIO`
        - `DigitalIO` ROS button object.
    - _btn_str : `str`
        - String value of the DigitalIO connection string used for activating
            the button.
    - comment : `str`
        - String value which contains a short comment of what the Button is.

    Methods
    -
    - _change_action(value) : `None`
        - State-change listener which is activated when the button is pressed
            or depressed.
    '''

    # Set of valid initialization values
    _VALID_INIT_PARAMS: Dict[str, Dict[str, List[str]]] = {
        'l': {
            'b': ['d1'],
            't': ['d1', 'd2', 'a'],
            'a': ['d1', 'd2', 'a'],
            'w': ['d1', 'd2', 'c'],
        },
        'r': {
            'b': ['d1'],
            't': ['d1', 'd2', 'a'],
            'a': ['d1', 'd2', 'a'],
            'w': ['d1', 'd2', 'c'],
        },
    }

    # ==============
    # Initialization
    def __init__(
            self,
            side: str,
            pos: str,
            sub_pos: str,
            comment: str = ''
    ) -> None:
        '''
        Robot Button Initialization
        -
        Initializes the Robot Button.

        Parameters
        -
        - side : `str`
            - Whether the button is on the left or right side of the robot.
            - Value values: `'l', 'r'`
                - 'l' = Left Side.
                - 'r' = Right Side.
        - pos : `str`
            - Whether the button is on the back, torso, arm, or wrist of the
                robot.
            - Valid values: `'b', 't', 'a', 'w'`
                - 'b' = Back.
                - 't' = Torso.
                - 'a' = Arm.
                - 'w' = Wrist.
        - sub_pos : `str`
            - Sub-position of the button relative to the `side` and `pos`
                parameters.
            - Valid values: `'d1', 'd2', 'a', 'c'`
                - For `pos == 'b'`, only valid value is 'd1'.
                - For all other positions, value 'd1' means closest to Baxter's 
                    head (or on the wrist it means the circle button), and 
                    value 'd2' is the other digital. The value 'a' means the
                    analogue wheel input for the torso and arm button sets.
                    The value 'c' means a digital read for the cuff squeeze.

        Returns
        -
        None
        '''

        # validating parameters
        if (
                (side not in self._VALID_INIT_PARAMS) \
                or (pos not in self._VALID_INIT_PARAMS[side]) \
                or (sub_pos not in self._VALID_INIT_PARAMS[side][pos])
        ):
            raise ValueError(
                f'Invalid initialziation value Button side={side}, pos={pos}' \
                    + f', sub_pos={sub_pos}'
            )
        
        # creating comment
        self.comment = comment
        
        # creating input string
        self._btn_str = {
            ('l', 'b', 'd1'): 'left_shoulder_button',
            ('l', 't', 'd1'): 'torso_left_button_back',
            ('l', 't', 'd2'): 'torso_left_button_show',
            ('l', 't', 'a'): 'torso_left_button_ok',
            ('l', 'a', 'd1'): 'left_button_back',
            ('l', 'a', 'd2'): 'left_button_show',
            ('l', 'a', 'a'): 'left_button_ok',
            ('l', 'w', 'd1'): 'left_lower_button',
            ('l', 'w', 'd2'): 'left_upper_button',
            ('l', 'w', 'c'): 'left_lower_cuff',
            ('r', 'b', 'd1'): 'right_shoulder_button',
            ('r', 't', 'd1'): 'torso_right_button_back',
            ('r', 't', 'd2'): 'torso_right_button_show',
            ('r', 't', 'a'): 'torso_right_button_ok',
            ('r', 'a', 'd1'): 'right_button_back',
            ('r', 'a', 'd2'): 'right_button_show',
            ('r', 'a', 'a'): 'right_button_ok',
            ('r', 'w', 'd1'): 'right_lower_button',
            ('r', 'w', 'd2'): 'right_upper_button',
            ('r', 'w', 'c'): 'right_lower_cuff',
        }[(side, pos, sub_pos)]

        # creating button
        self._btn = DigitalIO(self._btn_str)

        # creating state-change listener functions
        self._btn.state_changed.connect(self._change_action)

    # =====================
    # State-Change Listener
    def _change_action(self, value: bool) -> None:
        '''
        State-Change Listener
        -
        State-change listener which is activated when the button is pressed or
        depressed.

        Parameters
        -
        - value : `bool`
            - Whether the button was pressed or depressed. Pressed = True.

        Returns
        -
        None
        '''

        print(f'Comment: {self.comment}, DigitalIO: {self._btn_str}, Value: {value}')

class Robot():
    def __init__(self):
        # Back/Shoulder Buttons
        self.back_left = Button('l', 'b', 'd1', 'Back Left')
        self.back_right = Button('r', 'b', 'd1', 'Back Right')

        # Torso Buttons
        self.torso_Ld1 = Button('l', 't', 'd1', 'Torso D1 Left')
        self.torso_Ld2 = Button('l', 't', 'd2', 'Torso D2 Left')
        self.torso_La = Button('l', 't', 'a', 'Torso Knob Btn Left')
        self.torso_Rd1 = Button('r', 't', 'd1', 'Torso D1 Right')
        self.torso_Rd2 = Button('r', 't', 'd2', 'Torso D2 Right')
        self.torso_Ra = Button('r', 't', 'a', 'Torso Knob Btn Right')
        
        # Arm Buttons
        self.arm_Ld1 = Button('l', 'a', 'd1', 'Arm D1 Left')
        self.arm_Ld2 = Button('l', 'a', 'd2', 'Arm D2 Left')
        self.arm_La = Button('l', 'a', 'a', 'Arm Knob Btn Left')
        self.arm_Rd1 = Button('r', 'a', 'd1', 'Arm D1 Right')
        self.arm_Rd2 = Button('r', 'a', 'd2', 'Arm D2 Right')
        self.arm_Ra = Button('r', 'a', 'a', 'Arm Knob Btn Right')

        # Wrist Buttons
        self.wrist_l1 = Button('l', 'w', 'd1', 'Wrist D1 Left')
        self.wrist_l2 = Button('l', 'w', 'd2', 'Wrist D2 Left')
        self.wrist_r1 = Button('r', 'w', 'd1', 'Wrist D1 Right')
        self.wrist_r2 = Button('r', 'w', 'd2', 'Wrist D2 Right')
        self.cuff_left = Button('l', 'w', 'c', 'Wrist Cuff Left')
        self.cuff_right = Button('r', 'w', 'c', 'Wrist Cuff Right')


# =============================================================================
# Main Loop
# =============================================================================
def main():
    rospy.init_node('baxter_controller', anonymous = True)
    bot = Robot()
    rospy.spin()
    print('Done')
    return 0

if __name__ == '__main__':
    main()


# =============================================================================
# End of File
# =============================================================================
