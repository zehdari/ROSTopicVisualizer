import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64MultiArray, Bool, Int32, String, UInt8
import array

class MultiParamNode(Node):
    def __init__(self):
        super().__init__('multi_param_node')

        # Declare parameters
        self.declare_parameter('double_array_param', [1.0, 2.0, 3.0])
        self.declare_parameter('int_param', 42)
        self.declare_parameter('string_param', 'Hello, ROS 2!')

        # Initialize publishers
        self.double_array_publisher_ = self.create_publisher(Float64MultiArray, 'double_array_topic', 10)
        self.int_publisher_ = self.create_publisher(Int32, 'int_topic', 10)
        self.string_publisher_ = self.create_publisher(String, 'string_topic', 10)

        # Timer to periodically publish messages
        self.timer = self.create_timer(1.0, self.publish_parameters)

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Log the initial parameter values
        self.log_initial_params()

    def log_initial_params(self):
        double_array = self.get_parameter('double_array_param').get_parameter_value().double_array_value
        int_value = self.get_parameter('int_param').get_parameter_value().integer_value
        string_value = self.get_parameter('string_param').get_parameter_value().string_value

        self.get_logger().info(f"Initial double_array_param: {list(double_array)}")
        self.get_logger().info(f"Initial int_param: {int_value}")
        self.get_logger().info(f"Initial string_param: {string_value}")

    def publish_parameters(self):
        # Publish each parameter type
        double_array = self.get_parameter('double_array_param').get_parameter_value().double_array_value
        int_value = self.get_parameter('int_param').get_parameter_value().integer_value
        string_value = self.get_parameter('string_param').get_parameter_value().string_value

        # Publish double array
        msg = Float64MultiArray()
        msg.data = list(double_array)
        self.double_array_publisher_.publish(msg)

        # Publish int value
        int_msg = Int32()
        int_msg.data = int_value
        self.int_publisher_.publish(int_msg)

        # Publish string value
        string_msg = String()
        string_msg.data = string_value
        self.string_publisher_.publish(string_msg)

        # Log the published values
        self.get_logger().info(f"Published double_array_param: {list(double_array)}")
        self.get_logger().info(f"Published int_param: {int_value}")
        self.get_logger().info(f"Published string_param: {string_value}")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'double_array_param' and param.type_ == Parameter.Type.DOUBLE_ARRAY:
                updated_array = param.value
                self.get_logger().info(f"Parameter 'double_array_param' updated to: {list(updated_array)}")
            elif param.name == 'int_param' and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f"Parameter 'int_param' updated to: {param.value}")
            elif param.name == 'string_param' and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"Parameter 'string_param' updated to: {param.value}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = MultiParamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MultiParamNode has been stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
