import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import ByteMultiArray

class ByteArrayNode(Node):
    def __init__(self):
        super().__init__('byte_array_node')

        # Default byte values (as integers) -> [65, 66, 67] ('A', 'B', 'C')
        default_values = ["01001010", "00001000", "00000000"]  # Integer list
        self.declare_parameter('byte_array_param', default_values)

        # Initial value retrieval
        byte_array = self.get_byte_array('byte_array_param')
        self.get_logger().info(f"Initial byte_array_param: {list(byte_array)}")

        # Publisher for byte array
        self.publisher_ = self.create_publisher(ByteMultiArray, 'byte_array_topic', 10)

        # Timer to publish data at 1-second intervals
        self.timer = self.create_timer(1.0, self.publish_byte_array)

        # Callback to handle parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def get_byte_array(self, param_name):
        # Retrieves the byte array from the parameter value
        param = self.get_parameter(param_name)
        if param.type_ == Parameter.Type.INTEGER_ARRAY:
            # Ensure we get a list of integers and convert them to bytearray
            return bytearray(param.value)
        else:
            self.get_logger().error(f"Parameter '{param_name}' is not an integer array!")
            return bytearray()  # Return an empty byte array if not found

    def publish_byte_array(self):
        # Publish the byte array to the topic
        byte_array = self.get_byte_array('byte_array_param')

        # Convert bytearray to bytes before assigning it to the message
        msg = ByteMultiArray()
        msg.data = bytes(byte_array)  # Convert bytearray to bytes

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published byte_array_param: {list(byte_array)}")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'byte_array_param' and param.type_ == Parameter.Type.INTEGER_ARRAY:
                updated_array = bytearray(param.value)
                self.get_logger().info(f"Parameter 'byte_array_param' updated to: {list(updated_array)}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ByteArrayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ByteArrayNode has been stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
