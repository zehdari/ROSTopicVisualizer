import os
from flask import Flask, request, jsonify
from flask_cors import CORS
import subprocess
import signal
from flask_socketio import SocketIO, emit
import pty
import select
import termios
import struct
import fcntl

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")
CORS(app, origins="*", methods=["GET", "POST", "OPTIONS"])

# Dictionary to store the running video server processes and their topics
video_server_processes = {}

# Dictionary to store terminal sessions
terminal_sessions = {}

def discover_ros_topics():
    try:
        command = "ros2 topic list"
        bash_command = f"bash -c 'source /home/ubuntu/.bashrc && {command}'"
        result = subprocess.run(bash_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            return [topic for topic in topics if topic]  # Filter out empty strings
        return []
    except Exception as e:
        print(f"Error discovering topics: {e}")
        return []

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

        # Get available topics and check if requested topic exists
        available_topics = discover_ros_topics()
        if topic not in available_topics:
            return jsonify({
                "error": f"Topic '{topic}' not found",
                "available_topics": available_topics
            }), 404

        # Stop the video server for the same topic if it's already running
        if topic in video_server_processes:
            stop_video_server(topic)

        command = f"run_as_superclient ros2 run web_video_server web_video_server --ros-args --param port:={port} --remap topic:={topic}"
        print(f"Running command: {command}")
        bash_command = f"bash -c 'source /home/ubuntu/.bashrc && {command}'"
        process = subprocess.Popen(bash_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        video_server_processes[topic] = process

        return jsonify({
            "message": f"Video server started for topic '{topic}' on port {port}.",
            "available_topics": available_topics
        })
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
    
@socketio.on('connect')
def handle_connect():
    print("Client connected")
    session_id = request.sid
    master_fd, slave_fd = pty.openpty()
    terminal_sessions[session_id] = {
        'master_fd': master_fd,
        'slave_fd': slave_fd,
        'shell': subprocess.Popen(
            ['bash'],
            preexec_fn=os.setsid,
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            universal_newlines=True,
            cwd=os.path.expanduser('~')
        )
    }
    fcntl.fcntl(master_fd, fcntl.F_SETFL, os.O_NONBLOCK)
    emit('connection_established', {'session_id': session_id})

@socketio.on('disconnect')
def handle_disconnect():
    session_id = request.sid
    if session_id in terminal_sessions:
        session = terminal_sessions[session_id]
        os.kill(session['shell'].pid, signal.SIGTERM)
        os.close(session['master_fd'])
        os.close(session['slave_fd'])
        del terminal_sessions[session_id]

@socketio.on('terminal_input')
def handle_terminal_input(data):
    session_id = request.sid
    if session_id in terminal_sessions:
        session = terminal_sessions[session_id]
        os.write(session['master_fd'], data['input'].encode())

def read_terminal_output():
    while True:
        for session_id, session in list(terminal_sessions.items()):
            try:
                master_fd = session['master_fd']
                ready, _, _ = select.select([master_fd], [], [], 0)
                if ready:
                    output = os.read(master_fd, 1024).decode(errors='replace')
                    socketio.emit('terminal_output', {'output': output}, room=session_id)
            except (OSError, IOError) as e:
                if e.errno == 5:  # Input/output error, terminal probably closed
                    continue
                print(f"Error reading terminal output: {e}")
        socketio.sleep(0.01)

@socketio.on('resize')
def handle_resize(data):
    session_id = request.sid
    if session_id in terminal_sessions:
        session = terminal_sessions[session_id]
        winsize = struct.pack("HHHH", data['rows'], data['cols'], 0, 0)
        fcntl.ioctl(session['master_fd'], termios.TIOCSWINSZ, winsize)

if __name__ == '__main__':
    socketio.start_background_task(read_terminal_output)
    socketio.run(app, host='0.0.0.0', port=5001, allow_unsafe_werkzeug=True)