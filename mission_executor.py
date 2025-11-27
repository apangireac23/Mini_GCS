#!/usr/bin/env python3

from pymavlink import mavutil
import time
import os

# Import your Pixhawk connector
from pixhawkconnection import MavlinkAutoConnector

class MissionExecutor:
    def __init__(self, connector: MavlinkAutoConnector):
        """Initialize using an existing Pixhawk connection manager"""
        if not connector.connection:
            raise ConnectionError("No connection from Pixhawk connector")
        self.master = connector.connection
        print(f"Successfully connected to vehicle "
              f"(system: {self.master.target_system}, component: {self.master.target_component})")

    def load_waypoints(self, filepath):
        """Load waypoints from QGC waypoint file"""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Mission file not found: {filepath}")

        print(f"Loading mission from: {filepath}")
        waypoints = []

        with open(filepath, 'r') as f:
            first_line = f.readline().strip()
            if not first_line.startswith("QGC WPL"):
                raise ValueError("Invalid mission file format. Must be QGC waypoint file.")

            for line in f:
                parts = line.strip().split('\t')
                if len(parts) == 12:
                    waypoint = {
                        'seq': int(parts[0]),
                        'current': int(parts[1]),
                        'frame': int(parts[2]),
                        'command': int(parts[3]),
                        'param1': float(parts[4]),
                        'param2': float(parts[5]),
                        'param3': float(parts[6]),
                        'param4': float(parts[7]),
                        'x': float(parts[8]),
                        'y': float(parts[9]),
                        'z': float(parts[10]),
                        'autocontinue': int(parts[11])
                    }
                    waypoints.append(waypoint)

        print(f"Loaded {len(waypoints)} waypoints")
        return waypoints

    def upload_mission(self, waypoints):
        """Upload mission waypoints to vehicle"""
        print("Starting mission upload...")
        self.master.mav.mission_clear_all_send(
            self.master.target_system,
            self.master.target_component
        )
        time.sleep(1)

        waypoint_count = len(waypoints)
        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            waypoint_count
        )

        for _ in range(waypoint_count):
            msg = self.master.recv_match(type=['MISSION_REQUEST'], blocking=True)
            if msg is None:
                raise Exception("Failed to receive mission request")

            wp = waypoints[msg.seq]
            print(f"Sending waypoint {msg.seq}")
            self.master.mav.mission_item_send(
                self.master.target_system,
                self.master.target_component,
                msg.seq,
                wp['frame'],
                wp['command'],
                wp['current'],
                wp['autocontinue'],
                wp['param1'],
                wp['param2'],
                wp['param3'],
                wp['param4'],
                wp['x'],
                wp['y'],
                wp['z']
            )

        msg = self.master.recv_match(type=['MISSION_ACK'], blocking=True)
        if msg and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"Successfully uploaded {waypoint_count} waypoints")
        else:
            raise Exception("Mission upload failed")

    def arm_and_execute(self):
        """Arm the vehicle and start mission execution with heartbeat debug"""

        def print_heartbeat_state(hb):
            armed_state = "ARMED" if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "DISARMED"
            mode_map = self.master.mode_mapping()
            current_mode = None
            for name, num in mode_map.items():
                if hb.custom_mode == num:
                    current_mode = name
                    break
            print(f"[HEARTBEAT] System: {hb.get_srcSystem()}  Armed: {armed_state}  Mode: {current_mode}  Status: {hb.system_status}")

        print("Starting vehicle arming and mission execution...")

        # Step 1: Switch to GUIDED mode
        guided_mode = self.master.mode_mapping().get('GUIDED')
        if guided_mode is None:
            raise Exception("GUIDED mode not supported by this vehicle")

        print("Setting mode to GUIDED...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_mode
        )

        timeout = time.time() + 10
        while time.time() < timeout:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb:
                print_heartbeat_state(hb)
                if hb.custom_mode == guided_mode:
                    print("Vehicle mode set to GUIDED")
                    break
        else:
            raise Exception("Failed to set GUIDED mode")

        # Step 2: Arm vehicle with retries
        print("Arming vehicle...")
        armed = False
        timeout = time.time() + 10
        while time.time() < timeout:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.1)
            if hb:
                print_heartbeat_state(hb)
                if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    armed = True
                    print("Vehicle armed")
                    break

        if not armed:
            raise Exception("Failed to arm vehicle (timed out)")

        # Step 3: Switch to AUTO mode
        auto_mode = self.master.mode_mapping().get('AUTO')
        if auto_mode is None:
            raise Exception("AUTO mode not supported by this vehicle")

        print("Switching to AUTO mode...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            auto_mode
        )

        timeout = time.time() + 10
        while time.time() < timeout:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb:
                print_heartbeat_state(hb)
                if hb.custom_mode == auto_mode:
                    print("Vehicle mode set to AUTO")
                    break
        else:
            raise Exception("Failed to set AUTO mode")

        # Step 4: Start mission
        print("Starting mission...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("Mission started")

