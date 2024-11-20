from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import rclpy
from rclpy.node import Node
from threading import Thread
from ros_topics import TOPICS  # Import topics and message types

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")


# ROS 2 Node for dynamic subscriptions
class ROS2WebStreamer(Node):
    def __init__(self):
        super().__init__('ros2_web_streamer')
        self.subscriptions_dict = {}  # Avoid name conflict with `rclpy.Node`

    def add_subscription(self, topic_name, msg_type):
        if topic_name in self.subscriptions_dict:
            self.get_logger().info(f"Already subscribed to topic: {topic_name}")
            return

        self.subscriptions_dict[topic_name] = self.create_subscription(
            msg_type,
            topic_name,
            lambda msg, t=topic_name: self.listener_callback(msg, t),
            10
        )
        self.get_logger().info(f"Subscribed to topic: {topic_name}")

    def listener_callback(self, msg, topic_name):
        msg_dict = self.message_to_dict(msg)
        socketio.emit('ros2_data', {'topic': topic_name, 'data': msg_dict})

    def message_to_dict(self, msg):
        # Convert ROS 2 message to a dictionary
        if hasattr(msg, '__slots__') and hasattr(msg, '_fields_and_field_types'):
            result = {}
            for field in msg.__slots__:
                value = getattr(msg, field)
                result[field] = self.message_to_dict(value)  # Recursively process fields
            return result
        elif isinstance(msg, (list, tuple)):  # Handle arrays
            return [self.message_to_dict(v) for v in msg]
        elif isinstance(msg, (int, float, str, bool, type(None))):  # Basic types
            return msg
        else:
            return str(msg)  # Fallback for unrecognized types


@app.route('/')
def index():
    return render_template('index.html')


# Run ROS 2 in a separate thread
def ros2_thread():
    rclpy.init()
    streamer = ROS2WebStreamer()

    # Add subscriptions from TOPICS
    for topic in TOPICS:
        streamer.add_subscription(topic['name'], topic['type'])

    rclpy.spin(streamer)
    streamer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    Thread(target=ros2_thread).start()
    socketio.run(app, host='0.0.0.0', port=5000)
