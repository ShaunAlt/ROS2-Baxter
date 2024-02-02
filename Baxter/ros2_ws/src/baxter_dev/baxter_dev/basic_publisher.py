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
from geometry_msgs.msg import ( # type: ignore
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
                        x = 0.6380249604294951,
                        y = 0.14025225230336114,
                        z = 0.12495353404139961
                    ),
                    orientation = Quaternion(
                        x = 0.18058038877281843,
                        y = 0.9806283842483902,
                        z = -0.015517398241163896,
                        w = 0.07428260596448302
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
