#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter Server / Client
-
Contains a function which provides a service for a new topic, and then
continuously polls clients to that topic
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================
from array import array
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from baxter_int_ros2_support.srv import ( # type: ignore
    CameraData
)
from typing import Any


# =============================================================================
# Topic Service
# =============================================================================
class MinimalService(Node):
    def __init__(self) -> None:
        super().__init__('minimal_subscriber')
        self.srv = self.create_service(
            CameraData,
            'service_camera_data',
            self.srv_callback
        )

    def srv_callback(
            self,
            request: CameraData.Request,
            response: CameraData.Response
    ) -> CameraData.Response:
        self.get_logger().info(
            f'MinimalService Callback'
        )
        response.img_data = array('B', [1,2,3])
        response.width = 1
        response.height = 1
        response.step = 3
        return response


# =============================================================================
# Topic Client
# =============================================================================
class MinimalClient(Node):
    def __init__(self, i: int) -> None:
        super().__init__(f'minimal_client_{i:03}')
        self.cli = self.create_client(
            CameraData,
            'service_camera_data'
        )
        while not self.cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again ...')
        self.req = CameraData.Request()
# class MinimalSubscriber(Node):
#     def __init__(self) -> None:
#         super().__init__('minimal_subscriber')
#         self.subscription = self.create_subscription(
#             DigitalIOState,
#             '/robot/digital_io/left_lower_cuff/state',
#             self.listener_callback,
#             10
#         )

#     def listener_callback(self, msg: DigitalIOState) -> None:
#         self.get_logger().info(
#             f'state = {msg.state}, is_input_only = {msg.is_input_only}'
#         )


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
