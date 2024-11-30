import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # Create publishers for two video topics
        self.publisher_left = self.create_publisher(Image, 'video_topic', 10)
        self.publisher_right = self.create_publisher(Image, 'video_topic2', 10)

        # Create a CvBridge object to convert OpenCV images to ROS messages
        self.bridge = CvBridge()

        # Open two video files using OpenCV
        self.cap_left = cv2.VideoCapture('/home/ubuntu/svo_converted_left_2024-02-10-03-21-54.avi')
        self.cap_right = cv2.VideoCapture('/home/ubuntu/svo_converted_right_2024-02-10-03-21-54.avi')

        if not self.cap_left.isOpened() or not self.cap_right.isOpened():
            self.get_logger().error("Failed to open video files.")
            return

        # Set a timer to publish the video frames at a fixed rate (e.g., 15 FPS)
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)

    def timer_callback(self):
        # Read a frame from both video captures
        ret_left, frame_left = self.cap_left.read()
        ret_right, frame_right = self.cap_right.read()

        if not ret_left or not ret_right:
            self.get_logger().info("End of video stream. Restarting...")
            self.cap_left.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart left video loop
            self.cap_right.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart right video loop
            return

        # Convert the frames to ROS Image messages
        try:
            ros_image_left = self.bridge.cv2_to_imgmsg(frame_left, encoding="bgr8")
            ros_image_right = self.bridge.cv2_to_imgmsg(frame_right, encoding="bgr8")

            # Publish the frames to respective topics
            self.publisher_left.publish(ros_image_left)
            self.publisher_right.publish(ros_image_right)

            self.get_logger().info("Publishing video frames.")
        except Exception as e:
            self.get_logger().error(f"Failed to convert frame to ROS message: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Create and spin the VideoPublisher node
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)

    # Clean up
    video_publisher.cap_left.release()
    video_publisher.cap_right.release()
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
