from geometry_msgs.msg import Twist  # Import message types
from std_msgs.msg import Bool, Float64
from nav_msgs.msg import Odometry

# List of topics with their message types
TOPICS = [
    {'name': '/turtle1/cmd_vel', 'type': Twist},  # Example topic
    {'name': '/turtle_enabled', 'type': Bool},
    {'name': '/sin_wave', 'type': Float64},
    {'name': '/odom', 'type': Odometry}
    # Add more topics as needed
]
