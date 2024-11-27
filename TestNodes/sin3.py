import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time


class SineWavePublisher(Node):
    def __init__(self):
        super().__init__('sin_wave_publisher')
        self.declare_parameter('topic_name', '/sin_wave3')
        self.declare_parameter('frequency', 1.0)  # Frequency of the sine wave in Hz
        self.declare_parameter('amplitude', 1.0)  # Amplitude of the sine wave

        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Float64, self.topic_name, 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_sine_wave)  # 30 Hz update rate
        self.start_time = time.time()

        self.get_logger().info(
            f"Sine wave publisher started on topic '{self.topic_name}' with "
            f"frequency={self.frequency} Hz, amplitude={self.amplitude}."
        )

    def publish_sine_wave(self):
        current_time = time.time() - self.start_time
        sine_value = self.amplitude * math.sin(2 * math.pi * self.frequency * current_time)
        msg = Float64()
        msg.data = sine_value
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {sine_value:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
