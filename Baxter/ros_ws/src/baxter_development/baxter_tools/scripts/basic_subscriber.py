#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter Subscriber
-
Contains a function which subscribes to a given topic and prints out all of the
values for that particular topic.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================
import rospy
from std_msgs.msg import String
from baxter_core_msgs.msg import (
    DigitalIOState
)
from typing import Any


# =============================================================================
# Message Callback Function
# =============================================================================
def msg_callback(data: DigitalIOState) -> None:
    # rospy.loginfo(
    #     f'Caller: {rospy.get_caller_id()} | state = {data.state}' \
    #         + f', isInputOnly = {data.isInputOnly}'
    # )
    rospy.loginfo(
        f'state = {data.state}, isInputOnly = {data.isInputOnly}'
    )

# =============================================================================
# Listener
# =============================================================================
def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber(
        '/robot/digital_io/left_lower_cuff/state', 
        DigitalIOState, 
        msg_callback
    )
    rospy.spin()


if __name__ == '__main__':
    listener()


# =============================================================================
# End of File
# =============================================================================
