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
from baxter_interface_msgs.msg import (
    EndpointTarget,
    EndpointTargets, 
)
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)
from typing import Any


# =============================================================================
# Message Publisher
# =============================================================================
class MinimalPublisher(Node):
    def __init__(self) -> None:
        super().__init__('minimal_publisher')
        self.topic = '/baxter_ros2/endpoint_targets'
        self.publisher_ = self.create_publisher(
            EndpointTargets,
            self.topic,
            10
        )
        self.create_timer(
            1.0,
            self.timer_callback
        )

    def timer_callback(self) -> None:
        # msg = EndpointTargets()
        # msg.limbs = [EndpointTargets.LIMB_L]
        # target1 = EndpointTarget()
        # pose = Pose()
        # pt = Point()
        # q = Quaternion()
        msg = EndpointTargets(
            limbs = [EndpointTargets.LIMB_L],
            targets = [EndpointTarget(
                pose = Pose(
                    position = Point(
                        x = 1.0,
                        y = 1.0,
                        z = 1.0
                    ),
                    orientation = Quaternion(
                        x = 1.0,
                        y = 1.0,
                        z = 1.0,
                        w = 1.0
                    )
                ),
                mode = EndpointTarget.MODE_NORMAL
            )]
        )
        self.publisher_.publish(msg)
        print(f'Published Message to {self.topic}')

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
