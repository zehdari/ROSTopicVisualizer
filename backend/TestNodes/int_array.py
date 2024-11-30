import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int32MultiArray
import array

class IntArrayNode(Node):
    def __init__(self):
        super().__init__('int_array_node')

        # Default integer values
        default_values = array.array('i', [1, 2, 3])  # Default as int32
        self.declare_parameter('int_array_param', list(default_values))

        # Initial value retrieval
        int_array = self.get_int32_array('int_array_param')
        self.get_logger().info(f"Initial int_array_param: {list(int_array)}")

        # Publisher for int array
        self.publisher_ = self.create_publisher(Int32MultiArray, 'int_array_topic', 10)

        # Timer to publish data at 1-second intervals
        self.timer = self.create_timer(1.0, self.publish_int_array)

        # Callback to handle parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def get_int32_array(self, param_name):
        # Retrieves the integer array from parameter
        values = self.get_parameter(param_name).get_parameter_value().integer_array_value
        return array.array('i', values)  # Convert to int32

    def publish_int_array(self):
        # Publish the int array to the topic
        int_array = self.get_int32_array('int_array_param')

        msg = Int32MultiArray()
        msg.data = int_array

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published int_array_param: {list(int_array)}")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'int_array_param' and param.type_ == Parameter.Type.INTEGER_ARRAY:
                updated_array = array.array('i', param.value)
                self.get_logger().info(f"Parameter 'int_array_param' updated to: {list(updated_array)}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = IntArrayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("IntArrayNode has been stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
