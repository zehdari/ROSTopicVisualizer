import os
from flask import Flask, request, jsonify
from flask_cors import CORS
import subprocess
import signal

app = Flask(__name__)

# Allow cross-origin requests from React app (running on localhost:3000)
CORS(app, origins="*", methods=["GET", "POST", "OPTIONS"])

# Dictionary to store the running video server processes and their topics
video_server_processes = {}

# Function to stop a video server process
def stop_video_server(topic):
    if topic in video_server_processes:
        process = video_server_processes[topic]
        try:
            process.terminate()  # Attempt to terminate the process gracefully
            process.wait()  # Wait for process termination
            print(f"Video server stopped for topic '{topic}'.")
        except Exception as e:
            print(f"Error stopping video server for topic '{topic}': {e}")
        finally:
            del video_server_processes[topic]  # Remove the process from the dictionary

@app.route('/start-video-server', methods=['POST'])
def start_video_server():
    try:
        data = request.get_json()
        print(f"Received data: {data}")
        
        topic = data.get('topic')
        port = data.get('port')

        if not topic or not port:
            return jsonify({"error": "Topic and port are required"}), 400

        # Stop the video server for the same topic if it's already running
        if topic in video_server_processes:
            stop_video_server(topic)

        command = f"ros2 run web_video_server web_video_server --ros-args --param port:={port} --remap topic:={topic}"
        print(f"Running command: {command}")

        bash_command = f"bash -c 'source /home/ubuntu/.bashrc && {command}'"
        
        process = subprocess.Popen(bash_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        video_server_processes[topic] = process  # Store the process in the dictionary with the topic as the key

        return jsonify({"message": f"Video server started for topic '{topic}' on port {port}."})

    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/stop-video-server', methods=['POST'])
def stop_video_server_route():
    try:
        data = request.get_json()
        print(f"Received data: {data}")
        
        topic = data.get('topic')

        if not topic:
            return jsonify({"error": "Topic is required"}), 400

        if topic in video_server_processes:
            stop_video_server(topic)
            print(f"video_server_processes: {video_server_processes}")
            return jsonify({"message": f"Video server stopped for topic '{topic}'."})
        else:
            return jsonify({"message": f"No video server running for topic '{topic}'."})
        
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)