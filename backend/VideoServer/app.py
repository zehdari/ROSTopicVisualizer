import os
from flask import Flask, request, jsonify
from flask_cors import CORS
import subprocess
import signal

app = Flask(__name__)

# Allow cross-origin requests from React app (running on localhost:3000)
CORS(app, origins="*", methods=["GET", "POST", "OPTIONS"])

# Global variable to hold the current video server process
current_process = None

# Function to stop the current video server if it's running
def stop_video_server():
    global current_process
    if current_process:
        try:
            current_process.terminate()  # Attempt to terminate the process gracefully
            current_process.wait()  # Wait for process termination
            print("Video server stopped.")
        except Exception as e:
            print(f"Error stopping video server: {e}")
        finally:
            current_process = None  # Reset the process reference

@app.route('/start-video-server', methods=['POST'])
def start_video_server():
    global current_process
    try:
        # Print out the received data for debugging
        data = request.get_json()
        print(f"Received data: {data}")
        
        topic = data.get('topic')
        port = data.get('port')

        # Ensure topic and port are provided
        if not topic or not port:
            return jsonify({"error": "Topic and port are required"}), 400

        # Stop the current video server if it's running
        if current_process:
            stop_video_server()

        # Build the command to run the web_video_server with the specified topic and port
        command = f"ros2 run web_video_server web_video_server --ros-args --param port:={port} --remap topic:={topic}"
        print(f"Running command: {command}")

        # Ensure ROS 2 is sourced before running the command
        bash_command = f"bash -c 'source /home/ubuntu/.bashrc && {command}'"  # Ensure the ROS 2 setup is sourced
        
        # Run the video server command in the background using subprocess.Popen
        current_process = subprocess.Popen(bash_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        return jsonify({"message": f"Video server started for topic '{topic}' on port {port}."})

    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)  # Make sure Flask is listening on all interfaces
