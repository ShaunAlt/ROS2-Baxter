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
# import rospy
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from baxter_core_msgs.msg import ( # type: ignore
    DigitalIOState
)
from typing import Any


# =============================================================================
# Message Subscriber
# =============================================================================
class MinimalSubscriber(Node):
    def __init__(self) -> None:
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            DigitalIOState,
            '/robot/digital_io/left_lower_cuff/state',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: DigitalIOState) -> None:
        self.get_logger().info(
            f'state = {msg.state}, is_input_only = {msg.is_input_only}'
        )


# =============================================================================
# Main Function
# =============================================================================
def main(args: None = None) -> None:
    rclpy.init(args = args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# =============================================================================
# End of File
# =============================================================================
