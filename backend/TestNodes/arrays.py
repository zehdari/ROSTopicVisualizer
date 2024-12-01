import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import (
    Float64MultiArray, Int32MultiArray, Float32MultiArray,
    Int8MultiArray, Int16MultiArray, Int64MultiArray,
    UInt8MultiArray, UInt16MultiArray, UInt32MultiArray, UInt64MultiArray
)

class CompleteArrayNode(Node):
    def __init__(self):
        super().__init__('complete_array_node')
        
        # Initialize publishers dictionary
        self.array_publishers = {}
        
        # Declare parameters for all array types
        self.declare_parameter('float64_array', [1.0, 2.0, 3.0])
        self.declare_parameter('float32_array', [1.0, 2.0, 3.0])
        self.declare_parameter('int8_array', [1, 2, 3])
        self.declare_parameter('int16_array', [1, 2, 3])
        self.declare_parameter('int32_array', [1, 2, 3])
        self.declare_parameter('int64_array', [1, 2, 3])
        self.declare_parameter('uint8_array', [1, 2, 3])
        self.declare_parameter('uint16_array', [1, 2, 3])
        self.declare_parameter('uint32_array', [1, 2, 3])
        self.declare_parameter('uint64_array', [1, 2, 3])
        
        # Create publishers for all array types
        self.array_publishers['float64'] = self.create_publisher(Float64MultiArray, 'float64_array', 10)
        self.array_publishers['float32'] = self.create_publisher(Float32MultiArray, 'float32_array', 10)
        self.array_publishers['int8'] = self.create_publisher(Int8MultiArray, 'int8_array', 10)
        self.array_publishers['int16'] = self.create_publisher(Int16MultiArray, 'int16_array', 10)
        self.array_publishers['int32'] = self.create_publisher(Int32MultiArray, 'int32_array', 10)
        self.array_publishers['int64'] = self.create_publisher(Int64MultiArray, 'int64_array', 10)
        self.array_publishers['uint8'] = self.create_publisher(UInt8MultiArray, 'uint8_array', 10)
        self.array_publishers['uint16'] = self.create_publisher(UInt16MultiArray, 'uint16_array', 10)
        self.array_publishers['uint32'] = self.create_publisher(UInt32MultiArray, 'uint32_array', 10)
        self.array_publishers['uint64'] = self.create_publisher(UInt64MultiArray, 'uint64_array', 10)
        
        # Timer to periodically publish messages
        self.timer = self.create_timer(1.0, self.publish_parameters)
        
        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Log initial parameters
        self.log_initial_params()

    def log_initial_params(self):
        for param_name in self.array_publishers.keys():
            param_value = self.get_parameter(f'{param_name}_array').get_parameter_value()
            if param_name.startswith(('float', 'double')):
                value = param_value.double_array_value
            else:
                value = param_value.integer_array_value
            self.get_logger().info(f"Initial {param_name}_array: {list(value)}")

    def publish_parameters(self):
        for param_type, publisher in self.array_publishers.items():
            param_name = f'{param_type}_array'
            param_value = self.get_parameter(param_name).get_parameter_value()
            
            # Create appropriate message type
            msg_type = publisher.msg_type
            msg = msg_type()

            # Handle all other types
            if param_type.startswith(('float', 'double')):
                value = param_value.double_array_value
            else:
                value = param_value.integer_array_value
            msg.data = list(value)
            
            # Publish message
            publisher.publish(msg)
            self.get_logger().info(f"Published {param_name}: {list(value)}")

    def parameter_callback(self, params):
        for param in params:
            param_base = param.name.replace('_array', '')
            if param_base in self.array_publishers:
                if param.type_ in [Parameter.Type.DOUBLE_ARRAY, Parameter.Type.INTEGER_ARRAY]:
                    self.get_logger().info(f"Parameter '{param.name}' updated to: {list(param.value)}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = CompleteArrayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("CompleteArrayNode has been stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()