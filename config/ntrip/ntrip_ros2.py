#!/usr/bin/env python3
"""
ROS2 NTRIP Client for MicroStrain GQ7
Publishes RTCM corrections to /rtcm topic as rtcm_msgs/msg/Message
Sends GGA position feedback for VRS mountpoints
"""

import socket
import base64
import time
import os
import signal
import math
import termios
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rtcm_msgs.msg import Message as RTCMMessage
from sensor_msgs.msg import NavSatFix

# Configuration from environment
NTRIP_HOST = os.environ.get('NTRIP_HOST', 'uk.nrtk.eu')
NTRIP_PORT = int(os.environ.get('NTRIP_PORT', '7801'))
NTRIP_MOUNTPOINT = os.environ.get('NTRIP_MOUNTPOINT', 'MSM_iMAX')
NTRIP_USER = os.environ.get('NTRIP_USER', '')
NTRIP_PASS = os.environ.get('NTRIP_PASS', '')
GGA_INTERVAL = float(os.environ.get('GGA_INTERVAL', '10'))

# Default approximate position (central UK) for initial VRS connection
DEFAULT_LAT = float(os.environ.get('DEFAULT_LAT', '52.0'))
DEFAULT_LON = float(os.environ.get('DEFAULT_LON', '-1.5'))
DEFAULT_ALT = float(os.environ.get('DEFAULT_ALT', '100.0'))

# Direct serial port for RTCM corrections to GQ7 aux GNSS receiver
AUX_SERIAL_PORT = os.environ.get('AUX_SERIAL_PORT', '')
AUX_SERIAL_BAUD = int(os.environ.get('AUX_SERIAL_BAUD', '921600'))


class NtripClient(Node):
    def __init__(self):
        super().__init__('ntrip_client')

        self.publisher = self.create_publisher(RTCMMessage, '/rtcm', 10)

        # Subscribe to GNSS fix for GGA feedback to VRS caster
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(NavSatFix, '/gnss_1/fix', self._gnss_cb, sensor_qos)
        self.create_subscription(NavSatFix, '/gnss_1/llh_position', self._gnss_cb, sensor_qos)

        self.sock = None
        self.running = True
        self.bytes_received = 0
        self.messages_published = 0
        self.last_stats = time.time()
        self.last_gga_sent = 0.0
        self.gnss_updates = 0

        # Start with default position, update when GNSS provides real fix
        self.latest_lat = DEFAULT_LAT
        self.latest_lon = DEFAULT_LON
        self.latest_alt = DEFAULT_ALT
        self.have_real_fix = False

        # Open direct serial port to GQ7 aux GNSS if configured
        self.aux_serial_fd = None
        if AUX_SERIAL_PORT:
            try:
                self.aux_serial_fd = os.open(AUX_SERIAL_PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                attrs = termios.tcgetattr(self.aux_serial_fd)
                attrs[0] = 0  # iflag: raw
                attrs[1] = 0  # oflag: raw
                attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
                attrs[3] = 0  # lflag: raw
                attrs[4] = termios.B921600  # ispeed
                attrs[5] = termios.B921600  # ospeed
                attrs[6][termios.VMIN] = 0
                attrs[6][termios.VTIME] = 0
                termios.tcsetattr(self.aux_serial_fd, termios.TCSANOW, attrs)
                self.aux_serial_bytes = 0
            except Exception as e:
                self.aux_serial_fd = None
                self.get_logger().error(f'Failed to open aux serial {AUX_SERIAL_PORT}: {e}')

        self.get_logger().info('=' * 50)
        self.get_logger().info('BESS NTRIP Client (ROS2)')
        self.get_logger().info(f'  Server: {NTRIP_HOST}:{NTRIP_PORT}')
        self.get_logger().info(f'  Mount:  {NTRIP_MOUNTPOINT}')
        self.get_logger().info(f'  Topic:  /rtcm (rtcm_msgs/msg/Message)')
        if self.aux_serial_fd is not None:
            self.get_logger().info(f'  Serial: {AUX_SERIAL_PORT} @ {AUX_SERIAL_BAUD}')
        self.get_logger().info(f'  GGA interval: {GGA_INTERVAL}s')
        self.get_logger().info(f'  Default pos: {DEFAULT_LAT:.4f}, {DEFAULT_LON:.4f}')
        self.get_logger().info('=' * 50)

    def _gnss_cb(self, msg):
        """Update latest position from GNSS receiver."""
        if msg.latitude != 0.0 and msg.longitude != 0.0:
            self.latest_lat = msg.latitude
            self.latest_lon = msg.longitude
            self.latest_alt = msg.altitude
            self.gnss_updates += 1
            if not self.have_real_fix:
                self.have_real_fix = True
                self.get_logger().info(
                    f'Got GNSS fix: {msg.latitude:.6f}, {msg.longitude:.6f}, alt={msg.altitude:.1f}m'
                )

    def _build_gga(self):
        """Build NMEA GGA sentence from latest GNSS position."""
        if self.latest_lat is None:
            return None

        now = datetime.now(timezone.utc)
        utc_time = now.strftime('%H%M%S.00')

        lat = abs(self.latest_lat)
        lat_deg = int(lat)
        lat_min = (lat - lat_deg) * 60.0
        lat_str = f'{lat_deg:02d}{lat_min:010.7f}'
        lat_dir = 'N' if self.latest_lat >= 0 else 'S'

        lon = abs(self.latest_lon)
        lon_deg = int(lon)
        lon_min = (lon - lon_deg) * 60.0
        lon_str = f'{lon_deg:03d}{lon_min:010.7f}'
        lon_dir = 'E' if self.latest_lon >= 0 else 'W'

        # Quality 1=GPS fix, 4=RTK fixed, 5=RTK float
        body = f'GPGGA,{utc_time},{lat_str},{lat_dir},{lon_str},{lon_dir},1,12,1.0,{self.latest_alt:.1f},M,0.0,M,,'
        checksum = 0
        for c in body:
            checksum ^= ord(c)
        return f'${body}*{checksum:02X}\r\n'

    def _send_gga(self, force=False):
        """Send GGA to NTRIP caster if interval elapsed."""
        now = time.time()
        if not force and (now - self.last_gga_sent < GGA_INTERVAL):
            return
        gga = self._build_gga()
        if gga and self.sock:
            try:
                self.sock.send(gga.encode())
                self.last_gga_sent = now
                fix_type = 'GNSS' if self.have_real_fix else 'default'
                self.get_logger().info(
                    f'Sent GGA ({fix_type}): {self.latest_lat:.6f}, {self.latest_lon:.6f}'
                )
            except Exception as e:
                self.get_logger().warn(f'Failed to send GGA: {e}')

    def connect_ntrip(self):
        """Connect to NTRIP caster"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(10)

        self.get_logger().info(f'Connecting to {NTRIP_HOST}:{NTRIP_PORT}/{NTRIP_MOUNTPOINT}...')
        self.sock.connect((NTRIP_HOST, NTRIP_PORT))

        auth = base64.b64encode(f"{NTRIP_USER}:{NTRIP_PASS}".encode()).decode()

        # Build initial GGA for the request
        gga = self._build_gga() or ''

        request = (
            f"GET /{NTRIP_MOUNTPOINT} HTTP/1.1\r\n"
            f"Host: {NTRIP_HOST}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP BESSClient/1.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"\r\n"
        )

        self.sock.send(request.encode())

        response = b""
        while b"\r\n\r\n" not in response:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise Exception("Connection closed while reading header")
            response += chunk

        header, remainder = response.split(b"\r\n\r\n", 1)
        header_str = header.decode()

        if "200 OK" in header_str or "ICY 200 OK" in header_str:
            self.get_logger().info(f'Connected to NTRIP caster: {NTRIP_MOUNTPOINT}')
            # Use short timeout so spin_once runs frequently
            self.sock.settimeout(1.0)

            # Send GGA immediately after connecting
            self._send_gga(force=True)

            return remainder
        else:
            raise Exception(f"NTRIP connection failed: {header_str}")

    def publish_rtcm(self, data):
        """Publish RTCM data to ROS topic and write to aux serial port"""
        msg = RTCMMessage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "rtcm"
        msg.message = list(data)
        self.publisher.publish(msg)
        self.messages_published += 1

        # Write RTCM directly to GQ7 aux serial port
        if self.aux_serial_fd is not None:
            try:
                os.write(self.aux_serial_fd, data)
                self.aux_serial_bytes += len(data)
            except OSError as e:
                self.get_logger().warn(f'Aux serial write failed: {e}')

    def run(self):
        """Main loop"""
        reconnect_delay = 5

        while self.running and rclpy.ok():
            try:
                remainder = self.connect_ntrip()

                if remainder:
                    self.publish_rtcm(remainder)
                    self.bytes_received += len(remainder)

                reconnect_delay = 5

                while self.running and rclpy.ok():
                    try:
                        # Process ROS callbacks (for GNSS position updates)
                        rclpy.spin_once(self, timeout_sec=0)

                        try:
                            data = self.sock.recv(4096)
                        except socket.timeout:
                            # Short timeout - just send GGA and continue
                            self._send_gga()
                            continue

                        if not data:
                            self.get_logger().warn('NTRIP connection closed by server')
                            break

                        self.publish_rtcm(data)
                        self.bytes_received += len(data)

                        # Send GGA back to VRS caster
                        self._send_gga()

                        now = time.time()
                        if now - self.last_stats > 30:
                            fix_status = 'GNSS' if self.have_real_fix else 'default'
                            serial_info = f', serial: {self.aux_serial_bytes:,}B' if self.aux_serial_fd else ''
                            self.get_logger().info(
                                f'RTCM: {self.bytes_received:,} bytes, '
                                f'{self.messages_published} msgs, '
                                f'GNSS updates: {self.gnss_updates}, '
                                f'pos: {fix_status}{serial_info}'
                            )
                            self.last_stats = now

                    except socket.timeout:
                        # Send GGA even during quiet periods
                        self._send_gga()
                        continue

            except Exception as e:
                self.get_logger().error(f'Error: {e}')

            finally:
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
                    self.sock = None

            if self.running and rclpy.ok():
                self.get_logger().info(f'Reconnecting in {reconnect_delay}s...')
                time.sleep(reconnect_delay)
                reconnect_delay = min(reconnect_delay * 2, 60)

    def shutdown(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass


def main():
    rclpy.init()
    node = NtripClient()

    def signal_handler(sig, frame):
        node.get_logger().info('Shutting down...')
        node.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
