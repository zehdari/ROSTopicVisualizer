import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import BoolMultiArray

class BoolArrayNode(Node):
    def __init__(self):
        super().__init__('bool_array_node')

        # Default boolean values
        default_values = [True, False, True]
        self.declare_parameter('bool_array_param', default_values)

        # Initial value retrieval
        bool_array = self.get_bool_array('bool_array_param')
        self.get_logger().info(f"Initial bool_array_param: {bool_array}")

        # Publisher for boolean array
        self.publisher_ = self.create_publisher(BoolMultiArray, 'bool_array_topic', 10)

        # Timer to publish data at 1-second intervals
        self.timer = self.create_timer(1.0, self.publish_bool_array)

        # Callback to handle parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def get_bool_array(self, param_name):
        # Retrieves the boolean array from parameter
        return self.get_parameter(param_name).get_parameter_value().bool_array_value

    def publish_bool_array(self):
        # Publish the boolean array to the topic
        bool_array = self.get_bool_array('bool_array_param')

        msg = BoolMultiArray()
        msg.data = bool_array

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published bool_array_param: {bool_array}")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'bool_array_param' and param.type_ == Parameter.Type.BOOL_ARRAY:
                updated_array = param.value
                self.get_logger().info(f"Parameter 'bool_array_param' updated to: {updated_array}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = BoolArrayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("BoolArrayNode has been stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
