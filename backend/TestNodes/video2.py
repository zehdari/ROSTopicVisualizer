import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # Create a publisher for the Image topic
        self.publisher_ = self.create_publisher(Image, 'video_topic2', 10)

        # Create a CvBridge object to convert OpenCV images to ROS messages
        self.bridge = CvBridge()

        # Open a video file using OpenCV
        self.cap = cv2.VideoCapture('/home/ubuntu/svo_converted_right_2024-02-10-03-21-54.avi')

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video file.")
            return

        # Set a timer to publish the video frames at a fixed rate (e.g., 30 FPS)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        # Read a frame from the video capture
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video stream.")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart video loop
            return

        # Convert the frame to a ROS Image message
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(ros_image)
            self.get_logger().info("Publishing video frame.")
        except Exception as e:
            self.get_logger().error(f"Failed to convert frame to ROS message: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Create and spin the VideoPublisher node
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)

    # Clean up
    video_publisher.cap.release()
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
