import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time

from action_tut.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        fb = Fibonacci.Feedback()
        fb.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            # seq.append(seq[i] + seq[i-1])
            fb.partial_sequence.append(
                fb.partial_sequence[i] + fb.partial_sequence[i-1]
            )
            self.get_logger().info(f'Feedback: {fb.partial_sequence}')
            goal_handle.publish_feedback(fb)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = fb.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()