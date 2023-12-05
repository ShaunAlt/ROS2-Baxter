import rclpy
import rclpy.node

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        self.declare_parameter('param1', 'hello world')
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        param1 = self.get_parameter('param1').get_parameter_value().string_value
        self.get_logger().info(f'Param value: {param1}')
        new_param = rclpy.parameter.Parameter(
            'param1',
            rclpy.Parameter.Type.STRING,
            'new value lols'
        )
        all_new_params = [new_param]
        self.set_parameters(all_new_params)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()