#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter Publisher
-
Contains a function which publishes to a given topic
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================
# import rospy
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from baxter_core_msgs.msg import ( # type: ignore
    DigitalOutputCommand
)
from typing import Any


# =============================================================================
# Message Publisher
# =============================================================================
class MinimalPublisher(Node):
    def __init__(self) -> None:
        super().__init__('minimal_publisher')
        self.io_name = 'right_inner_light'
        self.io_val = False
        self.topic = '/robot/digital_io/command'
        self.publisher_ = self.create_publisher(
            DigitalOutputCommand,
            self.topic,
            10
        )
        self.create_timer(
            1.0,
            self.timer_callback
        )

    def timer_callback(self) -> None:
        msg = DigitalOutputCommand()
        msg.name = self.io_name
        msg.value = self.io_val
        self.io_val = not self.io_val
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing to "{self.topic}", name="{msg.name}", value = ' \
                + f'{msg.value}'
        )


# =============================================================================
# Main Function
# =============================================================================
def main(args: None = None) -> None:
    rclpy.init(args = args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# =============================================================================
# End of File
# =============================================================================
