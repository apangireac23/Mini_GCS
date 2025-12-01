import threading
import time
from pymavlink import mavutil
from pixhawkconnection import MavlinkAutoConnector

socketio = None

# ===============================
# 🔧 CONFIGURATION
# ===============================
EMIT_INTERVAL = 1.0         # seconds — how often to emit updates to frontend
RECONNECT_DELAY = 2.0       # seconds — delay before retrying connection
STREAM_RATE_HZ = 5           # Hz — how often Pixhawk sends telemetry (GLOBAL_POSITION_INT, etc.)
# ===============================

_last_gps_emit = 0
_last_alt_emit = 0
_last_state_emit = 0

_state = {
    "latitude": None,
    "longitude": None,
    "altitude": None,
    "armed": None,
    "connected": False,
    "mode": None,
}

class _MAVConn:
    def __init__(self, connector: MavlinkAutoConnector):
        if not connector.connection:
            raise ConnectionError("No connection from Pixhawk connector")
        self.master = connector.connection
        print(
            f"✅ Connected to vehicle "
            f"(system: {self.master.target_system}, component: {self.master.target_component})"
        )

    def connect(self):
        return bool(self.master)

    def recv(self, blocking=True, timeout=1):
        try:
            if not self.master:
                return None
            return self.master.recv_match(blocking=blocking, timeout=timeout)
        except Exception:
            return None


mav = None
streams_requested = False  # To ensure we only request once per connection


def _try_resolve_mode(custom_mode):
    try:
        mapping = mav.master.mode_mapping()
        for name, val in mapping.items():
            if val == custom_mode:
                return name
    except Exception:
        return None
    return None


def _request_data_streams(master):
    """📡 Request Pixhawk to start sending telemetry streams."""
    print(f"📡 Requesting data streams at {STREAM_RATE_HZ} Hz...")
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            STREAM_RATE_HZ,
            1,
        )
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            STREAM_RATE_HZ,
            1,
        )
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
            STREAM_RATE_HZ,
            1,
        )
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            STREAM_RATE_HZ,
            1,
        )
        print("✅ Stream requests sent successfully.")
    except Exception as e:
        print(f"⚠️ Failed to request streams: {e}")


def _emit_telemetry_if_due():
    global _last_gps_emit, _last_alt_emit, _last_state_emit
    now = time.time()

    to_emit = {}
    telemetry_keys = ("latitude", "longitude", "altitude")
    for k in telemetry_keys:
        if _state.get(k) is not None:
            to_emit[k] = _state[k]
    status = {}
    for k in ("armed", "connected", "mode"):
        if _state.get(k) is not None:
            status[k] = _state[k]

    if (now - _last_gps_emit) >= EMIT_INTERVAL:
        if "latitude" in to_emit or "longitude" in to_emit or "altitude" in to_emit:
            # Send all position data together
            position_data = {}
            if "latitude" in to_emit:
                position_data["latitude"] = to_emit["latitude"]
            if "longitude" in to_emit:
                position_data["longitude"] = to_emit["longitude"]
            if "altitude" in to_emit:
                position_data["altitude"] = to_emit["altitude"]
            
            if position_data:
                socketio.emit("position_update", position_data)
                _last_gps_emit = now
                _last_alt_emit = now  # Update both timestamps since we're sending together

    if (now - _last_state_emit) >= EMIT_INTERVAL:
        if status:
            socketio.emit("status", status)
            _last_state_emit = now


def _telemetry_loop():
    global mav, streams_requested
    connector = None

    while True:
        # Ensure we have a valid connection
        if mav is None or not getattr(mav, "master", None):
            try:
                connector = connector or MavlinkAutoConnector()
                if not connector.connect():
                    time.sleep(RECONNECT_DELAY)
                    continue
                connector.wait_heartbeat()
                mav = _MAVConn(connector)
                _state["connected"] = True
                streams_requested = False
            except Exception:
                _state["connected"] = False
                time.sleep(RECONNECT_DELAY)
                continue

        msg = mav.recv(blocking=True, timeout=1)
        if not msg:
            if not getattr(mav, "master", None):
                _state["connected"] = False
            time.sleep(0.01)
            continue

        t = msg.get_type()

        if t == "HEARTBEAT":
            try:
                _state["connected"] = True
                _state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                resolved = _try_resolve_mode(msg.custom_mode) if hasattr(msg, "custom_mode") else None
                _state["mode"] = resolved or getattr(msg, "custom_mode", None)

                # Request data streams once after connection
                if not streams_requested and mav and mav.master:
                    _request_data_streams(mav.master)
                    streams_requested = True
            except Exception as e:
                print(f"⚠️ HEARTBEAT handling error: {e}")

        elif t == "GLOBAL_POSITION_INT":
            try:
                if hasattr(msg, "lat") and hasattr(msg, "lon"):
                    _state["latitude"] = msg.lat / 1e7
                    _state["longitude"] = msg.lon / 1e7
                if hasattr(msg, "relative_alt"):
                    _state["altitude"] = (msg.relative_alt or 0) / 1000.0
                print(f"🌍 Position Update: lat={_state['latitude']}, lon={_state['longitude']}, alt={_state['altitude']}")
            except Exception as e:
                print(f"⚠️ Error parsing GLOBAL_POSITION_INT: {e}")

        try:
            if socketio:
                _emit_telemetry_if_due()
        except Exception:
            pass


def start_ros_listener(socket):
    global socketio
    socketio = socket

    thread = threading.Thread(target=_telemetry_loop, daemon=True)
    thread.start()


def get_state():
    """Return the latest known vehicle state."""
    return dict(_state)
