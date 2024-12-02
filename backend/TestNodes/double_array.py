import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64MultiArray
import array

class DoubleArrayNode(Node):
    def __init__(self):
        super().__init__('double_array_node')

        default_values = array.array('d', [1.0, 2.0, 3.0])  # Default as float64
        self.declare_parameter('double_array_param', list(default_values))

        double_array = self.get_float64_array('double_array_param')
        self.get_logger().info(f"Initial double_array_param: {list(double_array)}")

        self.publisher_ = self.create_publisher(Float64MultiArray, 'double_array_topic', 10)

        self.timer = self.create_timer(1.0, self.publish_double_array)

        self.add_on_set_parameters_callback(self.parameter_callback)

    def get_float64_array(self, param_name):
        values = self.get_parameter(param_name).get_parameter_value().double_array_value
        return array.array('d', values)  # Convert to float64

    def publish_double_array(self):
        double_array = self.get_parameter(param_name).get_parameter_value().double_array_value

        msg = Float64MultiArray()
        msg.data = double_array

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published double_array_param: {list(double_array)}")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'double_array_param' and param.type_ == Parameter.Type.DOUBLE_ARRAY:
                updated_array = array.array('d', param.value)
                self.get_logger().info(f"Parameter 'double_array_param' updated to: {list(updated_array)}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = DoubleArrayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("DoubleArrayNode has been stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
