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
    def __init__(self, connection_type='auto'):
        """
        Initialize the MAVLink connector
        
        Args:
            connection_type (str): Type of connection to use. Can be 'auto', 'serial', or 'udp'.
                                 If 'auto', will try both serial and UDP connections.
        """
        self.baudrates_to_try = [57600, 115200, 921600]
        self.connection = None
        self.connection_string = None
        self.baudrate = None
        self.port = None
        self.connection_type = connection_type.lower()
        self.udp_ports = [14550, 14551, 14540]  # Common UDP ports for SITL and PX4

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

    def connect_udp(self, udp_port=14550):
        """Attempt to connect via UDP"""
        try:
            connection_string = f'udp:127.0.0.1:{udp_port}'
            logger.info(f"Attempting UDP connection to {connection_string}")
            
            self.connection = mavutil.mavlink_connection(
                connection_string,
                input=True,
                source_system=255,
                source_component=1,
                autoreconnect=True,
                retries=3
            )
            
            # Test the connection
            try:
                self.connection.wait_heartbeat(timeout=5)
                self.connection_string = connection_string
                logger.info(f"✅ Connected to {connection_string}")
                return True
            except Exception as e:
                self.connection.close()
                self.connection = None
                logger.debug(f"No heartbeat on UDP port {udp_port}: {str(e)}")
                return False
                
        except Exception as e:
            logger.error(f"Error creating UDP connection: {str(e)}")
            return False

    def connect_serial(self):
        """Attempt to connect via serial port"""
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
        return False

    def connect(self, connection_type=None):
        """
        Attempt to connect to a MAVLink device
        
        Args:
            connection_type (str, optional): Override the connection type. Can be 'auto', 'serial', or 'udp'.
        """
        logger.debug("[DEBUG] Starting connect()...")
        
        # Use instance connection_type if not overridden
        connection_type = connection_type.lower() if connection_type else self.connection_type
        
        if connection_type == 'udp':
            # Try UDP connection
            for port in self.udp_ports:
                if self.connect_udp(port):
                    return True
            logger.error("❌ No working UDP connections found.")
            return False
            
        elif connection_type == 'serial':
            # Try serial connection
            if self.connect_serial():
                return True
            logger.error("❌ No working serial connections found.")
            return False
            
        else:  # auto
            # First try UDP (for SITL)
            for port in self.udp_ports:
                if self.connect_udp(port):
                    return True
            
            # Then try serial
            if self.connect_serial():
                return True
                
            logger.error("❌ No working connections found (tried UDP and serial).")
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


