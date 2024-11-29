import subprocess

def launch_rosbridge():
    try:
        # Run the ROS2 launch command
        subprocess.run(["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error launching ROS2 bridge: {e}")
    except KeyboardInterrupt:
        print("Script interrupted. Stopping ROS2 bridge.")

if __name__ == "__main__":
    launch_rosbridge()
