#!/usr/bin/env python3

import os
import sys
import socket
import threading
# Prefer eventlet for Raspberry Pi compatibility
try:
    import eventlet  # type: ignore
    eventlet.monkey_patch()
except Exception:
    pass

from datetime import datetime
from typing import Optional

from flask import Flask, render_template, request, redirect, url_for, jsonify, send_from_directory
from flask_socketio import SocketIO


from mission_executor import MissionExecutor
from pixhawkconnection import MavlinkAutoConnector
from mavlink_bridge import start_ros_listener, get_state

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
UPLOAD_DIR = os.path.join(BASE_DIR, 'uploads')
os.makedirs(UPLOAD_DIR, exist_ok=True)


app = Flask(__name__)
app.config['SECRET_KEY'] = 'ros-secret'
app.config['UPLOAD_FOLDER'] = UPLOAD_DIR

# Using threading async_mode to avoid hard dependency if eventlet/gevent unavailable at import time
# Prefer eventlet when available, but allow override and default to threading for safety
default_async_mode = os.environ.get('SOCKETIO_ASYNC_MODE') or ('eventlet' if 'eventlet' in sys.modules else 'threading')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode=default_async_mode, logger=True, engineio_logger=True)

# -----------------------------
# Log and print redirection
# -----------------------------
class SocketIOStdout:
    def __init__(self, namespace: str = '/'):
        self.namespace = namespace
        self._lock = threading.Lock()

    def write(self, message: str):
        if not message:
            return
        # Split to avoid huge buffers and drop empty lines
        for line in str(message).splitlines():
            text = line.strip('\n')
            if text:
                with self._lock:
                    socketio.emit('log', {'message': text}, namespace=self.namespace)

    def flush(self):
        # No-op to satisfy file-like interface
        pass


class StdoutRedirector:
    def __init__(self, new_stdout):
        self.new_stdout = new_stdout
        self._old_stdout = None

    def __enter__(self):
        self._old_stdout = sys.stdout
        sys.stdout = self.new_stdout
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old_stdout is not None:
            sys.stdout = self._old_stdout


# -----------------------------
# Global mission state
# -----------------------------
latest_mission_path_lock = threading.Lock()
latest_mission_path: Optional[str] = None
mission_thread: Optional[threading.Thread] = None


def set_latest_mission_path(path: Optional[str]):
    global latest_mission_path
    with latest_mission_path_lock:
        latest_mission_path = path


def get_latest_mission_path() -> Optional[str]:
    with latest_mission_path_lock:
        return latest_mission_path


# -----------------------------
# Routes
# -----------------------------
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/uploads/<path:filename>')
def download_file(filename):
    return send_from_directory(app.config['UPLOAD_FOLDER'], filename, as_attachment=True)


@app.route('/upload', methods=['POST'])
def upload():
    file = request.files.get('mission')
    if file is None or file.filename == '':
        return jsonify({'ok': False, 'error': 'No file provided'}), 400

    if not file.filename.lower().endswith('.waypoints'):
        return jsonify({'ok': False, 'error': 'Only .waypoints files are allowed'}), 400

    timestamp = datetime.utcnow().strftime('%Y%m%d_%H%M%S')
    safe_name = f"mission_{timestamp}.waypoints"
    save_path = os.path.join(app.config['UPLOAD_FOLDER'], safe_name)
    file.save(save_path)
    set_latest_mission_path(save_path)
    socketio.emit('log', {'message': f"Uploaded mission saved to: {save_path}"})
    return jsonify({'ok': True, 'path': save_path, 'filename': safe_name})


def run_mission_worker(mission_path: str):
    emitter = SocketIOStdout()
    with StdoutRedirector(emitter):
        print("Starting mission workflow...")
        try:
            connector = MavlinkAutoConnector()
            if not connector.connect():
                print("Error: Failed to connect to Pixhawk")
                return

            connector.wait_heartbeat()

            executor = MissionExecutor(connector)
            waypoints = executor.load_waypoints(mission_path)
            executor.upload_mission(waypoints)
            executor.arm_and_execute()

            print("Mission execution completed")
        except Exception as e:
            print(f"Error during mission: {e}")


@app.route('/start', methods=['POST'])
def start_mission():
    global mission_thread
    if mission_thread and mission_thread.is_alive():
        return jsonify({'ok': False, 'error': 'Mission already running'}), 409

    mission_path = get_latest_mission_path()
    if not mission_path or not os.path.exists(mission_path):
        return jsonify({'ok': False, 'error': 'No uploaded mission found'}), 400

    mission_thread = threading.Thread(target=run_mission_worker, args=(mission_path,), daemon=True)
    mission_thread.start()
    socketio.emit('log', {'message': 'Mission started in background'})
    return jsonify({'ok': True})


@socketio.on('connect')
def handle_connect():
    socketio.emit('log', {'message': 'Connected to server'})
    try:
        state = get_state()
        status = {k: state.get(k) for k in ("armed", "connected", "mode") if state.get(k) is not None}
        telem = {k: state.get(k) for k in ("latitude", "longitude", "altitude") if state.get(k) is not None}
        if status:
            socketio.emit('status', status)
        if telem:
            socketio.emit('telemetry', telem)
    except Exception:
        pass


def main():
    # Bind to all interfaces for LAN access on Pi
    host = os.environ.get('HOST', '0.0.0.0')
    port = int(os.environ.get('PORT', '5000'))
    debug = os.environ.get('FLASK_DEBUG', '0') == '1'
    start_ros_listener(socketio)
    print("[INFO] Starting Flask server...")
    socketio.run(app, host=host, port=port, debug=debug)


if __name__ == '__main__':
    main()


