import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(1 / 15, self.publish_odom)  # Publish at 15 Hz
        self.counter = 0.0  # Simulate motion

    def publish_odom(self):
        # Create an Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'

        # Simulate position and velocity
        odom.pose.pose.position.x = self.counter
        odom.pose.pose.position.y = self.counter * 0.5
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0  # Identity quaternion

        odom.twist.twist.linear.x = 1.0  # Moving forward
        odom.twist.twist.angular.z = 0.1  # Slight rotation

        # Publish the message
        self.publisher_.publish(odom)
        self.get_logger().info(f"Publishing Odometry: x={odom.pose.pose.position.x}, y={odom.pose.pose.position.y}")

        self.counter += 1.0 / 15  # Increment for the next publish (scaled for 15 Hz)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
