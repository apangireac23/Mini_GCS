#!/usr/bin/env python3
import os
import sys
import time
import logging
import serial
import serial.tools.list_ports
from typing import Optional
from pymavlink import mavutil

# Logging setup
logging.basicConfig(
    level=logging.DEBUG,  # DEBUG for full verbosity
    format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger(__name__)


class MavlinkAutoConnector:
    def __init__(self):
        self.baudrates_to_try = [57600, 115200, 921600]
        self.connection = None
        self.connection_string = None
        self.baudrate = None
        self.port = None

    def scan_serial_ports(self):
        """Scan for available serial ports"""
        logger.debug("[DEBUG] Scanning for serial ports...")
        ports = serial.tools.list_ports.comports()
        available_ports = [p.device for p in ports]
        logger.info(f"Found serial ports: {available_ports}")
        return available_ports

    def test_port_with_baudrates(self, port: str) -> Optional[int]:
        """Test a port with different baudrates"""
        logger.debug(f"[DEBUG] Starting test_port_with_baudrates for port: {port}")

        # Linux-style permission check: attempt to open the device file
        try:
            with open(port, 'rb') as _:
                logger.debug(f"[DEBUG] Successfully opened {port} for read test.")
            logger.info(f"✅ Permission check passed for {port}")
        except PermissionError:
            logger.error(
                f"❌ Permission denied for {port}. Try: sudo usermod -a -G dialout $USER && newgrp dialout"
            )
            return None
        except Exception as e:
            logger.error(f"❌ Cannot access {port}: {repr(e)}")
            return None

        # Test different baudrates
        for baudrate in self.baudrates_to_try:
            logger.info(f"Testing {port} at {baudrate} baud...")

            try:
                conn_str = f"{port}"
                logger.debug(f"[DEBUG] Attempting mavlink_connection to {conn_str} at baud {baudrate}")

                test_conn = mavutil.mavlink_connection(
                    conn_str,
                    baud=baudrate,
                    timeout=10,
                    autoreconnect=False
                )

                start_time = time.time()
                while time.time() - start_time < 8:
                    try:
                        msg = test_conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                        if msg:
                            logger.info(f"✅ SUCCESS! {port} works at {baudrate} baud")
                            logger.info(f"   System ID: {msg.get_srcSystem()}")
                            logger.info(f"   Component ID: {msg.get_srcComponent()}")
                            test_conn.close()
                            return baudrate
                    except Exception as e:
                        logger.debug(f"[DEBUG] Heartbeat recv error: {repr(e)}")
                        pass

                    elapsed = time.time() - start_time
                    logger.debug(f"[DEBUG] Waiting for heartbeat... {elapsed:.1f}s/8s")

                test_conn.close()
                logger.info(f"❌ No response at {baudrate} baud")

            except Exception as e:
                logger.error(f"[DEBUG] Error testing {port} at {baudrate}: {repr(e)}")

        return None

    def connect(self):
        """Attempt auto-connection to a MAVLink device"""
        logger.debug("[DEBUG] Starting connect()...")

        ports = self.scan_serial_ports()
        if not ports:
            logger.error("❌ No serial ports found.")
            return False

        for port in ports:
            baud = self.test_port_with_baudrates(port)
            if baud:
                self.port = port
                self.baudrate = baud
                self.connection_string = f"{port}:{baud}"
                logger.info(f"[DEBUG] Connection string set to: {self.connection_string}")

                try:
                    logger.debug(f"[DEBUG] Creating mavlink_connection to {self.port} at {self.baudrate}")
                    self.connection = mavutil.mavlink_connection(
                        self.port,
                        baud=self.baudrate,
                        timeout=10,
                        autoreconnect=True
                    )
                    logger.info(f"✅ Connected to {self.connection_string}")
                    return True
                except Exception as e:
                    logger.error(f"❌ Failed to establish persistent connection: {repr(e)}")
                    return False

        logger.error("❌ No working ports found.")
        return False

    def wait_heartbeat(self):
        """Wait for the first heartbeat"""
        logger.debug("[DEBUG] Waiting for heartbeat from connected device...")
        try:
            self.connection.wait_heartbeat(timeout=30)
            logger.info(f"✅ Heartbeat received from system {self.connection.target_system} "
                        f"component {self.connection.target_component}")
        except Exception as e:
            logger.error(f"❌ No heartbeat received: {repr(e)}")
            raise

    def monitor_messages(self):
        """Monitor incoming MAVLink messages"""
        logger.info("📡 Monitoring MAVLink messages (press Ctrl+C to exit)...")
        try:
            while True:
                msg = self.connection.recv_match(blocking=True, timeout=5)
                if msg:
                    logger.debug(f"[DEBUG] Received: {msg}")
        except KeyboardInterrupt:
            logger.info("🛑 Monitoring stopped by user.")
        except Exception as e:
            logger.error(f"❌ Error during message monitoring: {repr(e)}")


