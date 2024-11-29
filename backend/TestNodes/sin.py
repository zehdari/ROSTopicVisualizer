from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time


class SineWavePublisher(Node):
    def __init__(self):
        super().__init__('sin_wave_publisher')
        
        # Declare parameters
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('amplitude', 1.0)
        
        # Initialize parameter values
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        
        # Create publisher
        self.publisher_ = self.create_publisher(Float64, "/sin_wave", 10)
        
        # Create a timer for periodic publishing
        self.timer = self.create_timer(1.0 / 30.0, self.publish_sine_wave)  # 30 Hz update rate
        self.start_time = time.time()
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.get_logger().info(
            f"Sine wave publisher started on topic '/sin_wave' with "
            f"frequency={self.frequency} Hz, amplitude={self.amplitude}."
        )

    def publish_sine_wave(self):
        current_time = time.time() - self.start_time
        sine_value = self.amplitude * math.sin(2 * math.pi * self.frequency * current_time)
        msg = Float64()
        msg.data = sine_value
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {sine_value:.2f}")

    def on_set_parameters(self, params):
        # Update parameter values dynamically
        for param in params:
            if param.name == 'amplitude' and param.type_ == param.Type.DOUBLE:
                self.amplitude = param.value
                self.get_logger().info(f"Updated amplitude to: {self.amplitude}")
            elif param.name == 'frequency' and param.type_ == param.Type.DOUBLE:
                self.frequency = param.value
                self.get_logger().info(f"Updated frequency to: {self.frequency}")
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
