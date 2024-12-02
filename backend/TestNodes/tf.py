import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import time

class ComplexTFTreePublisher(Node):
    def __init__(self):
        super().__init__('complex_tf_tree_publisher')
        
        # Create a TransformBroadcaster instance
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to continuously publish transforms
        self.timer = self.create_timer(0.1, self.publish_transforms)

        # Some basic time variables to simulate time-based transforms
        self.frame_count = 0

    def publish_transforms(self):
        transforms = []

        # Frame 1: "base_link"
        t1 = geometry_msgs.msg.TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = "world"
        t1.child_frame_id = "base_link"
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        transforms.append(t1)
        
        # Frame 2: "arm_link"
        t2 = geometry_msgs.msg.TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = "base_link"
        t2.child_frame_id = "arm_link"
        t2.transform.translation.x = 1.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.5
        t2.transform.rotation.w = 0.866
        transforms.append(t2)
        
        # Frame 3: "forearm_link"
        t3 = geometry_msgs.msg.TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = "arm_link"
        t3.child_frame_id = "forearm_link"
        t3.transform.translation.x = 1.2
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.0
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.5
        t3.transform.rotation.w = 0.866
        transforms.append(t3)
        
        # Frame 4: "camera_link"
        t4 = geometry_msgs.msg.TransformStamped()
        t4.header.stamp = self.get_clock().now().to_msg()
        t4.header.frame_id = "forearm_link"
        t4.child_frame_id = "camera_link"
        t4.transform.translation.x = 0.2
        t4.transform.translation.y = 0.1
        t4.transform.translation.z = 0.5
        t4.transform.rotation.x = 0.0
        t4.transform.rotation.y = 0.0
        t4.transform.rotation.z = 0.0
        t4.transform.rotation.w = 1.0
        transforms.append(t4)
        
        # Frame 5: "lidar_link"
        t5 = geometry_msgs.msg.TransformStamped()
        t5.header.stamp = self.get_clock().now().to_msg()
        t5.header.frame_id = "base_link"
        t5.child_frame_id = "lidar_link"
        t5.transform.translation.x = 0.5
        t5.transform.translation.y = 0.0
        t5.transform.translation.z = 1.0
        t5.transform.rotation.x = 0.0
        t5.transform.rotation.y = 0.0
        t5.transform.rotation.z = 0.0
        t5.transform.rotation.w = 1.0
        transforms.append(t5)

        # Frame 6: "gripper_link"
        t6 = geometry_msgs.msg.TransformStamped()
        t6.header.stamp = self.get_clock().now().to_msg()
        t6.header.frame_id = "forearm_link"
        t6.child_frame_id = "gripper_link"
        t6.transform.translation.x = 0.1
        t6.transform.translation.y = 0.1
        t6.transform.translation.z = 0.0
        t6.transform.rotation.x = 0.0
        t6.transform.rotation.y = 0.0
        t6.transform.rotation.z = 0.707
        t6.transform.rotation.w = 0.707
        transforms.append(t6)
        
        # Frame 7: "gripper_finger_link"
        t7 = geometry_msgs.msg.TransformStamped()
        t7.header.stamp = self.get_clock().now().to_msg()
        t7.header.frame_id = "gripper_link"
        t7.child_frame_id = "gripper_finger_link"
        t7.transform.translation.x = 0.05
        t7.transform.translation.y = 0.0
        t7.transform.translation.z = 0.0
        t7.transform.rotation.x = 0.0
        t7.transform.rotation.y = 0.0
        t7.transform.rotation.z = 0.0
        t7.transform.rotation.w = 1.0
        transforms.append(t7)

        # Publish all transforms at once
        self.broadcaster.sendTransform(transforms)

        # Update frame count (just for visual feedback)
        self.frame_count += 1
        if self.frame_count > 100:  # reset after 100 iterations
            self.frame_count = 0


def main(args=None):
    rclpy.init(args=args)
    
    node = ComplexTFTreePublisher()

    # Spin the node to keep it active
    rclpy.spin(node)

    # Shutdown ROS 2 when done
    rclpy.shutdown()


if __name__ == '__main__':
    main()