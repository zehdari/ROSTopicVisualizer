import subprocess
import os
import sys
import signal
import time
from pathlib import Path

class ProcessLauncher:
    def __init__(self):
        self.processes = []
        self.base_path = Path(__file__).parent.resolve()
        
    def launch_all(self):
        # Define the order and paths of scripts to launch
        scripts = [
            ('run_rosbridge.py', 'Running ROS Bridge'),
            ('FlaskServer/app.py', 'Starting Flask Server'),
            ('TestNodes/arrays.py', 'Starting Arrays Node'),
            ('TestNodes/odom.py', 'Starting Odometry Node'),
            ('TestNodes/point_cloud.py', 'Starting Point Cloud Node'),
            ('TestNodes/sin.py', 'Starting Sin Node'),
            ('TestNodes/tf.py', 'Starting TF Node'),
            ('TestNodes/tf2.py', 'Starting 2nd TF Node'),
            ('TestNodes/turtle_kill_switch.py', 'Starting Turtle Kill Switch'),
            ('TestNodes/turtle_node.py', 'Starting Turtle Node'),
            ('TestNodes/video.py', 'Starting Video Node')
        ]
        
        # Launch each script
        for script_path, message in scripts:
            full_path = self.base_path / script_path
            if not full_path.exists():
                print(f"Warning: {script_path} not found!")
                continue
                
            print(message)
            try:
                process = subprocess.Popen([sys.executable, str(full_path)],
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE)
                self.processes.append(process)
                time.sleep(1)  # Give each process time to start
            except Exception as e:
                print(f"Error starting {script_path}: {e}")
    
    def shutdown(self):
        print("\nShutting down all processes...")
        for process in self.processes:
            try:
                process.terminate()
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        print("All processes terminated.")

def signal_handler(signum, frame):
    launcher.shutdown()
    sys.exit(0)

if __name__ == "__main__":
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    launcher = ProcessLauncher()
    print("Starting all nodes...")
    launcher.launch_all()
    
    try:
        # Keep the main process running
        while True:
            time.sleep(1)
            # Check if any process has terminated unexpectedly
            for process in launcher.processes:
                if process.poll() is not None:
                    print(f"Warning: A process has terminated unexpectedly!")
    except KeyboardInterrupt:
        launcher.shutdown()