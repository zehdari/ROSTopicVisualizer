import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg

class StereoCameraTFPublisher(Node):
    def __init__(self):
        super().__init__('stereo_camera_tf_publisher')
        
        # Create a TransformBroadcaster instance
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to continuously publish transforms
        self.timer = self.create_timer(0.1, self.publish_transforms)

    def publish_transforms(self):
        transforms = []
        current_time = self.get_clock().now().to_msg()

        # Left camera transform
        t_left = geometry_msgs.msg.TransformStamped()
        t_left.header.stamp = current_time
        t_left.header.frame_id = "camera_link"
        t_left.child_frame_id = "left_camera_link"
        # Position left camera 0.06m to the left of the center (typical stereo baseline is around 12cm)
        t_left.transform.translation.x = 0.0
        t_left.transform.translation.y = -0.06
        t_left.transform.translation.z = 0.0
        # Keep the same orientation as the center camera
        t_left.transform.rotation.x = 0.0
        t_left.transform.rotation.y = 0.0
        t_left.transform.rotation.z = 0.0
        t_left.transform.rotation.w = 1.0
        transforms.append(t_left)

        # Right camera transform
        t_right = geometry_msgs.msg.TransformStamped()
        t_right.header.stamp = current_time
        t_right.header.frame_id = "camera_link"
        t_right.child_frame_id = "right_camera_link"
        # Position right camera 0.06m to the right of the center
        t_right.transform.translation.x = 0.0
        t_right.transform.translation.y = 0.06
        t_right.transform.translation.z = 0.0
        # Keep the same orientation as the center camera
        t_right.transform.rotation.x = 0.0
        t_right.transform.rotation.y = 0.0
        t_right.transform.rotation.z = 0.0
        t_right.transform.rotation.w = 1.0
        transforms.append(t_right)

        # Publish all transforms at once
        self.broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    
    node = StereoCameraTFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()