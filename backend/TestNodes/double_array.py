# double_array_node_with_publisher_corrected.py

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray
import array  # For type conversion

class DoubleArrayNode(Node):
    def __init__(self):
        super().__init__('double_array_node')

        # Declare a parameter named 'double_array_param' with a default list of doubles
        self.declare_parameter('double_array_param', [1.0, 2.0, 3.0])

        # Get the parameter value
        double_array = self.get_parameter('double_array_param').get_parameter_value().double_array_value
        self.get_logger().info(f"Initial double_array_param: {double_array}")

        # Set up a publisher to publish the double array
        self.publisher_ = self.create_publisher(Float32MultiArray, 'double_array_topic', 10)

        # Timer to publish the parameter value periodically
        self.timer = self.create_timer(1.0, self.publish_double_array)

        # Set up a parameter change callback to handle dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def publish_double_array(self):
        # Retrieve the parameter value
        double_array = self.get_parameter('double_array_param').get_parameter_value().double_array_value

        # Convert 'd' array (float64) to 'f' array (float32)
        double_array_float32 = array.array('f', double_array)

        # Create and populate the message
        msg = Float32MultiArray()
        msg.data = double_array_float32

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published double_array_param: {list(double_array_float32)}")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'double_array_param' and param.type_ == Parameter.Type.DOUBLE_ARRAY:
                self.get_logger().info(f"Parameter 'double_array_param' updated to: {param.value}")
        return rclpy.parameter.SetParametersResult(successful=True)

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
