import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
import array

class StringArrayNode(Node):
    def __init__(self):
        super().__init__('string_array_node')

        # Default string values
        default_values = ['apple', 'banana', 'cherry']
        self.declare_parameter('string_array_param', default_values)

        # Initial value retrieval
        string_array = self.get_string_array('string_array_param')
        self.get_logger().info(f"Initial string_array_param: {string_array}")

        # Publisher for string array
        self.publisher_ = self.create_publisher(String, 'string_array_topic', 10)

        # Timer to publish data at 1-second intervals
        self.timer = self.create_timer(1.0, self.publish_string_array)

        # Callback to handle parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def get_string_array(self, param_name):
        # Retrieves the string array from parameter
        return self.get_parameter(param_name).get_parameter_value().string_array_value

    def publish_string_array(self):
        # Publish the string array to the topic
        string_array = self.get_string_array('string_array_param')

        msg = String()
        msg.data = ', '.join(string_array)  # Join list into a single string

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published string_array_param: {string_array}")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'string_array_param' and param.type_ == Parameter.Type.STRING_ARRAY:
                updated_array = param.value
                self.get_logger().info(f"Parameter 'string_array_param' updated to: {updated_array}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = StringArrayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("StringArrayNode has been stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
