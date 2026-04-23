#!/usr/bin/env python3
"""
FLIR A6701 Thermal Camera ROS2 Node using Spinnaker SDK

Selects camera by MAC address for resilience across IP changes / resets.
Sets GevSCPSPacketSize=1400 (cameras don't support jumbo frames).
Auto-reconnects on acquisition failure.

Usage:
    python3 thermal_spinnaker_node_patched.py --ros-args \
        -r __ns:=/thermal/camera1 \
        -p frame_id:=thermal_camera1_optical_frame \
        -p camera_mac:=00:11:1c:05:8f:20
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, Temperature
from std_msgs.msg import Header
import numpy as np
import socket
import struct
import time
import threading
import PySpin


# Raw GVCP probe constants — pre-Init gate to confirm camera MAC is GVCP-
# responsive BEFORE we trigger Spinnaker SDK enumeration. Two A6701 units
# (`:20` 2026-04-22 and `:4C` 2026-04-23) were lost following sequences
# that combined `docker stop/up` retry loops + MikroTik port bounces +
# Spinnaker re-enumeration cycles against an already-flapping iPORT.
# The fix is to NOT enumerate Spinnaker if the camera isn't already
# answering GVCP — single passive probe per cycle, no per-minute SDK kick.
GVCP_PORT = 3956
GVCP_KEY = 0x42
GVCP_FLAG_ACK_REQ = 0x01
GVCP_CMD_DISCOVERY = 0x0002
GVCP_SENSOR_SRC_IP = "169.254.100.1"


def _gvcp_passive_probe(target_mac_lower, timeout_s=2.0):
    """Send one GVCP DISCOVERY broadcast on the sensor subnet, return True
    if `target_mac_lower` (lowercased "aa:bb:cc:dd:ee:ff") appears in any
    reply. False on no reply or no match. Does not open a control session;
    DISCOVERY is read-only at the camera level. Single shot per call."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((GVCP_SENSOR_SRC_IP, 0))
        except OSError:
            # Not on sensor subnet — gate fails open (don't block) so we
            # don't accidentally disable cameras during dev outside Thor.
            return True
        s.settimeout(0.3)
        hdr = struct.pack(">BBHHH", GVCP_KEY, GVCP_FLAG_ACK_REQ,
                          GVCP_CMD_DISCOVERY, 0, 0xAA00 & 0xFFFF)
        s.sendto(hdr, ("255.255.255.255", GVCP_PORT))
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                data, _ = s.recvfrom(4096)
            except socket.timeout:
                continue
            # DISCOVERY_ACK has 0xF8 byte for MAC offset (RFC: bytes 18-23)
            if len(data) < 24:
                continue
            mac_bytes = data[18:24]
            mac_str = ":".join(f"{b:02x}" for b in mac_bytes)
            if mac_str == target_mac_lower:
                return True
        return False
    except Exception:
        # Any failure → gate fails open so we don't permanently block
        # cameras due to a probe bug.
        return True
    finally:
        try:
            s.close()
        except Exception:
            pass


class ThermalSpinnakerNode(Node):
    """ROS2 Node for FLIR A6701 using Spinnaker SDK."""

    def __init__(self):
        super().__init__('thermal_spinnaker_node')

        self.declare_parameter('frame_id', 'thermal_camera')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('strip_metadata_row', True)
        self.declare_parameter('publish_camera_info', True)
        self.declare_parameter('camera_mac', '')
        self.declare_parameter('packet_size', 1400)
        # GigE Vision heartbeat timeout is ~45s on A70 firmware; a 5s
        # reconnect is too short — the camera still holds the previous
        # control session and the new Open() fails with -1010. 60s gives
        # the camera time to fully release the session before we retry.
        self.declare_parameter('reconnect_interval', 60.0)
        # Rate watchdog: if the driver is publishing but the rate drops
        # below `rate_floor_hz` for `rate_floor_window_s` consecutive
        # seconds, force a reconnect even if consecutive_errors hasn't
        # tripped. The A70 degrades to sustained 3-8 Hz with one -1011
        # every ~80s (never 50 consecutive) and the driver otherwise
        # sits there forever at half rate. Set rate_floor_hz=0 to disable.
        self.declare_parameter('rate_floor_hz', 3.0)
        self.declare_parameter('rate_floor_window_s', 30.0)
        # How many consecutive errors before forcing a reconnect. Upstream
        # default was 50 but that rarely fires because successful frames
        # reset the counter. 15 is tight enough to catch sustained -1011
        # bursts without bouncing on single-packet drops.
        self.declare_parameter('error_threshold', 15)
        # Stuck-state back-off: if the camera goes through `stuck_threshold`
        # reconnect cycles without ever producing a Publishing-at-N-Hz
        # log line (i.e. all attempts hit -1010 / -1008 / -1002 / -1005
        # before any frame succeeds), assume Pleora firmware is in a
        # stuck control-session state. Switch to `stuck_backoff_s` sleep
        # instead of the normal reconnect_interval so the camera firmware
        # actually drains its dead heartbeat instead of being kicked
        # every minute. Counter resets to 0 on the first Publishing line.
        self.declare_parameter('stuck_threshold', 3)
        self.declare_parameter('stuck_backoff_s', 300.0)
        self.declare_parameter('binning', 1)
        self.declare_parameter('pixel_format', 'Mono16')
        self.declare_parameter('temperature_period_sec', 15.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.target_fps = self.get_parameter('frame_rate').value
        self.strip_metadata = self.get_parameter('strip_metadata_row').value
        self.publish_info = self.get_parameter('publish_camera_info').value
        self.camera_mac = self.get_parameter('camera_mac').value.lower().strip()
        self.packet_size = self.get_parameter('packet_size').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.rate_floor_hz = float(self.get_parameter('rate_floor_hz').value)
        self.rate_floor_window_s = float(self.get_parameter('rate_floor_window_s').value)
        self.error_threshold = int(self.get_parameter('error_threshold').value)
        self.stuck_threshold = int(self.get_parameter('stuck_threshold').value)
        self.stuck_backoff_s = float(self.get_parameter('stuck_backoff_s').value)
        self.binning = self.get_parameter('binning').value
        self.pixel_format = self.get_parameter('pixel_format').value
        self.temp_period = float(self.get_parameter('temperature_period_sec').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_pub = self.create_publisher(Image, 'image_raw', sensor_qos)
        if self.publish_info:
            self.info_pub = self.create_publisher(CameraInfo, 'camera_info', sensor_qos)

        # Temperature publishers. FLIR AX5-ICD thermal cameras (A6701, A70)
        # expose SensorTemperature (FPA) and HousingTemperature as read-only
        # CFloat Celsius nodes. Read mid-acquisition from the same thread as
        # GetNextImage so there's no PySpin nodemap race. Missing nodes are
        # tolerated — camera families without the feature just won't publish.
        self.fpa_temp_pub = self.create_publisher(Temperature, 'temperature', sensor_qos)
        self.housing_temp_pub = self.create_publisher(Temperature, 'housing_temperature', sensor_qos)

        self.system = None
        self.cam = None
        self.running = True
        self.width = 640
        self.height = 513
        self._last_temp_read = 0.0
        self._temp_nodes_missing = set()
        self._temp_discovery_done = False
        self._temp_node_units = {}
        self._temp_primary = None
        self._temp_housing = None
        # Stuck-state counter: incremented on every reconnect attempt that
        # didn't see a Publishing-at log line; reset on first publish.
        self._reconnects_without_publish = 0

        # Start main loop in a thread
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _get_cam_mac(self, cam):
        """Get MAC address string from a Spinnaker camera handle."""
        tl = cam.GetTLDeviceNodeMap()
        mac_node = PySpin.CIntegerPtr(tl.GetNode('GevDeviceMACAddress'))
        if not mac_node.IsValid():
            return ''
        mac_int = mac_node.GetValue()
        return ':'.join(f'{(mac_int >> (8 * j)) & 0xff:02x}' for j in range(5, -1, -1))

    def _get_cam_ip(self, cam):
        """Get IP address string from a Spinnaker camera handle."""
        tl = cam.GetTLDeviceNodeMap()
        ip_node = PySpin.CIntegerPtr(tl.GetNode('GevDeviceIPAddress'))
        if not ip_node.IsValid():
            return '?.?.?.?'
        v = ip_node.GetValue()
        return f'{(v >> 24) & 0xff}.{(v >> 16) & 0xff}.{(v >> 8) & 0xff}.{v & 0xff}'

    def _find_sensor_interface(self):
        """Find the Spinnaker interface for the sensor network (169.254.x.x)."""
        iface_list = self.system.GetInterfaces()
        sensor_iface = None
        for i in range(iface_list.GetSize()):
            iface = iface_list.GetByIndex(i)
            tl = iface.GetTLNodeMap()
            ip_node = PySpin.CIntegerPtr(tl.GetNode('GevInterfaceSubnetIPAddress'))
            if ip_node.IsValid() and PySpin.IsReadable(ip_node):
                ip_int = ip_node.GetValue()
                ip_str = f'{(ip_int>>24)&0xff}.{(ip_int>>16)&0xff}.{(ip_int>>8)&0xff}.{ip_int&0xff}'
                name_node = PySpin.CStringPtr(tl.GetNode('InterfaceDisplayName'))
                name = name_node.GetValue() if name_node.IsValid() else f'iface{i}'
                self.get_logger().info(f'  Interface {name}: {ip_str}')
                # Match the sensor subnet (169.254.100.1)
                if ip_str.startswith('169.254.'):
                    sensor_iface = iface
        iface_list.Clear()
        return sensor_iface

    def connect_camera(self) -> bool:
        """Connect to camera via Spinnaker, using sensor interface only."""
        try:
            # Pre-Init GVCP gate — confirm the camera's MAC is responding to
            # passive GVCP DISCOVERY BEFORE we initialise Spinnaker. If the
            # camera is in flap-state, dark on the wire, or bricked, we do
            # NOT want to keep triggering Spinnaker's internal interface-walk
            # enumeration (which sends GVCP control-channel packets to every
            # camera on the subnet on every Init). That cumulative SDK-init
            # traffic is what deepened the 2026-04-22/2026-04-23 A6701 brick
            # cycles. Single read-only DISCOVERY broadcast per cycle, ~2 s.
            if self.camera_mac:
                if not _gvcp_passive_probe(self.camera_mac.lower()):
                    self.get_logger().warn(
                        f'GVCP gate: {self.camera_mac} not responding to '
                        f'passive discovery — skipping Spinnaker Init this '
                        f'cycle (camera dark / flap-state / bricked)'
                    )
                    return False

            if self.system is None:
                self.system = PySpin.System.GetInstance()

            # Sensor-interface-only discovery. Host networking exposes every
            # NIC with a 169.254 neighbour to Spinnaker; System.GetCameras()
            # then enumerates the same camera once per reachable interface,
            # causing -1005 "open by another application" on a later Open()
            # because a ghost handle from the duplicate interface already
            # holds the session. Fix: pick the 169.254.* sensor interface
            # explicitly and enumerate only on that one Interface.
            # (See ros-drivers/flir_camera_driver#190, research notes 2026-04-21.)
            sensor_iface = self._find_sensor_interface()
            if sensor_iface is not None:
                cam_list = sensor_iface.GetCameras()
                self.get_logger().info(
                    f'Using sensor interface only ({cam_list.GetSize()} camera(s))'
                )
            else:
                self.get_logger().warn(
                    'Sensor interface (169.254.*) not found — falling back to global discovery'
                )
                cam_list = self.system.GetCameras()

            n_cams = cam_list.GetSize()
            self.get_logger().info(f'Spinnaker found {n_cams} camera(s)')

            self.cam = None
            for i in range(n_cams):
                cam = cam_list.GetByIndex(i)
                tl = cam.GetTLDeviceNodeMap()

                model_node = PySpin.CStringPtr(tl.GetNode('DeviceModelName'))
                if not (model_node.IsValid() and PySpin.IsReadable(model_node)):
                    continue
                model = model_node.GetValue()
                mac = self._get_cam_mac(cam)
                ip = self._get_cam_ip(cam)
                sn_node = PySpin.CStringPtr(tl.GetNode('DeviceSerialNumber'))
                sn = sn_node.GetValue() if (sn_node.IsValid() and PySpin.IsReadable(sn_node)) else '?'
                self.get_logger().info(f'  [{i}] {model}  mac={mac}  ip={ip}  sn={sn}')

                # Match by MAC if specified
                if self.camera_mac:
                    if mac == self.camera_mac:
                        self.cam = cam
                        self.get_logger().info(f'Matched by MAC: {mac}')
                        break
                else:
                    # Fallback: first matching model
                    if 'Xsc' in model or 'A6701' in model or 'A67' in model:
                        self.cam = cam
                        self.get_logger().info(f'Selected: {model} mac={mac}')
                        break

            if self.cam is None:
                target = f' (mac={self.camera_mac})' if self.camera_mac else ''
                self.get_logger().warn(f'No matching camera found{target}')
                cam_list.Clear()
                return False

            self.cam.Init()

            # Auto-discover max packet size for the network path, then cap to our target
            try:
                self.cam.DiscoverMaxPacketSize()
                discovered = self.cam.GevSCPSPacketSize.GetValue()
                self.get_logger().info(f'DiscoverMaxPacketSize: {discovered}')
                if discovered > self.packet_size:
                    self.cam.GevSCPSPacketSize.SetValue(self.packet_size)
                    self.get_logger().info(f'Capped packet size to {self.packet_size}')
            except Exception as e:
                self.get_logger().warn(f'DiscoverMaxPacketSize failed: {e}')
                # Fallback: set directly via QuickSpin
                try:
                    self.cam.GevSCPSPacketSize.SetValue(self.packet_size)
                    self.get_logger().info(f'Set GevSCPSPacketSize={self.packet_size} (QuickSpin)')
                except Exception as e2:
                    self.get_logger().error(f'Cannot set packet size: {e2}')

            # Configure stream channel AFTER Init() for reliable high-bandwidth capture
            try:
                tl_stream = self.cam.GetTLStreamNodeMap()

                # Enable packet resend (retransmit lost packets)
                resend = PySpin.CBooleanPtr(tl_stream.GetNode('StreamPacketResendEnable'))
                if resend.IsValid() and PySpin.IsWritable(resend):
                    resend.SetValue(True)
                    self.get_logger().info('Enabled packet resend')

                # Increase stream buffer count for large frames
                buf_mode = PySpin.CEnumerationPtr(tl_stream.GetNode('StreamBufferCountMode'))
                if buf_mode.IsValid() and PySpin.IsWritable(buf_mode):
                    manual = buf_mode.GetEntryByName('Manual')
                    if manual is not None:
                        buf_mode.SetIntValue(manual.GetValue())
                buf_count = PySpin.CIntegerPtr(tl_stream.GetNode('StreamBufferCountManual'))
                if buf_count.IsValid() and PySpin.IsWritable(buf_count):
                    buf_count.SetValue(10)
                    self.get_logger().info('Set stream buffer count=10')

                # Increase receive socket buffer
                sock_size = PySpin.CIntegerPtr(tl_stream.GetNode('SocketBufferSize'))
                if sock_size.IsValid() and PySpin.IsWritable(sock_size):
                    sock_size.SetValue(67108864)  # 64MB
                    self.get_logger().info('Set socket buffer=64MB')

                # Set packet resend timeout
                resend_timeout = PySpin.CIntegerPtr(tl_stream.GetNode('StreamPacketResendTimeout'))
                if resend_timeout.IsValid() and PySpin.IsWritable(resend_timeout):
                    resend_timeout.SetValue(10000)
                    self.get_logger().info('Set packet resend timeout=10000')

            except PySpin.SpinnakerException as e:
                self.get_logger().warn(f'Stream config: {e}')

            self.configure_camera()

            nodemap = self.cam.GetNodeMap()
            w = PySpin.CIntegerPtr(nodemap.GetNode('Width'))
            h = PySpin.CIntegerPtr(nodemap.GetNode('Height'))
            self.width = w.GetValue() if w.IsValid() else 640
            self.height = h.GetValue() if h.IsValid() else 513
            self.get_logger().info(f'Resolution: {self.width}x{self.height}')

            if self.strip_metadata and self.height == 513:
                self.get_logger().info('Will strip metadata row (513 -> 512)')

            return True

        except PySpin.SpinnakerException as e:
            self.get_logger().error(f'Spinnaker connection error: {e}')
            return False

    def configure_camera(self):
        """Configure camera for streaming."""
        nodemap = self.cam.GetNodeMap()

        # Log current camera settings before any changes
        try:
            pkt = PySpin.CIntegerPtr(nodemap.GetNode('GevSCPSPacketSize'))
            pf = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
            w = PySpin.CIntegerPtr(nodemap.GetNode('Width'))
            h = PySpin.CIntegerPtr(nodemap.GetNode('Height'))
            self.get_logger().info(
                f'Camera defaults: PacketSize={pkt.GetValue() if pkt.IsValid() else "?"} '
                f'PixelFormat={pf.GetCurrentEntry().GetSymbolic() if pf.IsValid() else "?"} '
                f'Resolution={w.GetValue() if w.IsValid() else "?"}x{h.GetValue() if h.IsValid() else "?"}'
            )
        except Exception as e:
            self.get_logger().warn(f'Could not read defaults: {e}')

        # Set GigE packet size — CRITICAL for network path compatibility
        try:
            pkt = PySpin.CIntegerPtr(nodemap.GetNode('GevSCPSPacketSize'))
            if pkt.IsValid() and PySpin.IsWritable(pkt):
                old = pkt.GetValue()
                pkt.SetValue(self.packet_size)
                self.get_logger().info(f'Set GevSCPSPacketSize: {old} -> {self.packet_size}')
            else:
                self.get_logger().error(
                    f'GevSCPSPacketSize NOT writable! valid={pkt.IsValid()} '
                    f'Current={pkt.GetValue() if pkt.IsValid() else "?"}'
                )
        except PySpin.SpinnakerException as e:
            self.get_logger().error(f'FAILED to set packet size: {e}')

        # Set DeviceLinkThroughputLimit to max (125 MB/s for GigE)
        try:
            dltl = PySpin.CIntegerPtr(nodemap.GetNode('DeviceLinkThroughputLimit'))
            if dltl.IsValid() and PySpin.IsWritable(dltl):
                old_tl = dltl.GetValue()
                dltl.SetValue(125000000)  # 125 MB/s = full GigE
                self.get_logger().info(f'Set DeviceLinkThroughputLimit: {old_tl} -> 125000000')
        except PySpin.SpinnakerException as e:
            self.get_logger().warn(f'Could not set DeviceLinkThroughputLimit: {e}')

        # Set GevSCPD (inter-packet delay) — 0 for max throughput on dedicated link
        try:
            scpd = PySpin.CIntegerPtr(nodemap.GetNode('GevSCPD'))
            if scpd.IsValid() and PySpin.IsWritable(scpd):
                scpd.SetValue(0)
                self.get_logger().info('Set GevSCPD=0 (max throughput)')
        except PySpin.SpinnakerException as e:
            self.get_logger().warn(f'Could not set GevSCPD: {e}')

        # Log stream channel destination for debugging
        try:
            dest = PySpin.CIntegerPtr(nodemap.GetNode('GevSCDA'))
            if dest.IsValid() and PySpin.IsReadable(dest):
                v = dest.GetValue()
                self.get_logger().info(
                    f'Stream destination: {(v>>24)&0xff}.{(v>>16)&0xff}.{(v>>8)&0xff}.{v&0xff}'
                )
            port = PySpin.CIntegerPtr(nodemap.GetNode('GevSCPHostPort'))
            if port.IsValid() and PySpin.IsReadable(port):
                self.get_logger().info(f'Stream port: {port.GetValue()}')
        except Exception as e:
            self.get_logger().warn(f'Could not read stream channel: {e}')

        # Set binning (always set to ensure camera state is correct)
        try:
            bh = PySpin.CIntegerPtr(nodemap.GetNode('BinningHorizontal'))
            bv = PySpin.CIntegerPtr(nodemap.GetNode('BinningVertical'))
            if bh.IsValid() and PySpin.IsWritable(bh):
                bh.SetValue(self.binning)
            if bv.IsValid() and PySpin.IsWritable(bv):
                bv.SetValue(self.binning)
            if self.binning > 1:
                self.get_logger().info(f'Set Binning={self.binning}x{self.binning}')
            else:
                self.get_logger().info('Binning disabled (1x1)')
        except PySpin.SpinnakerException as e:
            self.get_logger().warn(f'Could not set binning: {e}')

        # Set PixelFormat
        try:
            pf = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
            if pf.IsValid() and PySpin.IsWritable(pf):
                entry = pf.GetEntryByName(self.pixel_format)
                if entry is not None and PySpin.IsReadable(entry):
                    pf.SetIntValue(entry.GetValue())
                    self.get_logger().info(f'Set PixelFormat={self.pixel_format}')
        except PySpin.SpinnakerException as e:
            self.get_logger().warn(f'Could not set PixelFormat: {e}')

        # Set IRFormat to Radiometric
        try:
            ir = PySpin.CEnumerationPtr(nodemap.GetNode('IRFormat'))
            if ir.IsValid() and PySpin.IsWritable(ir):
                rad = ir.GetEntryByName('Radiometric')
                if rad is not None and PySpin.IsReadable(rad):
                    ir.SetIntValue(rad.GetValue())
                    self.get_logger().info('Set IRFormat=Radiometric')
        except PySpin.SpinnakerException as e:
            self.get_logger().warn(f'Could not set IRFormat: {e}')

        # Enable GevIEEE1588 (PTP clock sync) — host is grandmaster on mgbe0_0 PHC1.
        # When enabled, the camera's stream-channel timestamp is disciplined to
        # the PTP grandmaster so image acquisition times align with /ouster/points
        # and all other PTP-synced sensors to sub-microsecond precision.
        # Applies to A6701 + A70 + Blackfly alike (all GigE Vision 2.0).
        try:
            ptp_en = PySpin.CBooleanPtr(nodemap.GetNode('GevIEEE1588'))
            if ptp_en.IsValid() and PySpin.IsWritable(ptp_en):
                ptp_en.SetValue(True)
                self.get_logger().info('Enabled GevIEEE1588 (PTP clock sync)')
            else:
                self.get_logger().warn('GevIEEE1588 node not writable — PTP unavailable on this camera')
        except PySpin.SpinnakerException as e:
            self.get_logger().warn(f'Could not enable GevIEEE1588: {e}')

        # Log initial PTP status (will be "Initializing" at this point; check later)
        try:
            ptp_stat = PySpin.CEnumerationPtr(nodemap.GetNode('GevIEEE1588Status'))
            if ptp_stat.IsValid() and PySpin.IsReadable(ptp_stat):
                self.get_logger().info(
                    f'GevIEEE1588Status={ptp_stat.GetCurrentEntry().GetSymbolic()}'
                )
        except PySpin.SpinnakerException:
            pass

        # Confirm
        try:
            pf = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
            if pf.IsValid() and PySpin.IsReadable(pf):
                self.get_logger().info(
                    f'PixelFormat confirmed: {pf.GetCurrentEntry().GetSymbolic()}'
                )
        except Exception:
            pass

    def _read_float_node(self, nodemap, name):
        """Read a GenICam CFloat node by name. Returns float or None.
        Caches missing nodes to avoid log spam across reads."""
        if name in self._temp_nodes_missing:
            return None
        try:
            node = PySpin.CFloatPtr(nodemap.GetNode(name))
            if not PySpin.IsAvailable(node) or not PySpin.IsReadable(node):
                self._temp_nodes_missing.add(name)
                self.get_logger().info(f'{name}: not available on this camera (skipping)')
                return None
            return float(node.GetValue())
        except PySpin.SpinnakerException:
            self._temp_nodes_missing.add(name)
            return None

    def _publish_temp_msg(self, pub, celsius):
        """Publish a sensor_msgs/Temperature reading with a fresh header stamp."""
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.temperature = celsius
        msg.variance = 0.0
        pub.publish(msg)

    # Feature-name substrings that identify read-only sensor temperature
    # readings (not scene config, not calibration constants).
    TEMP_FEATURE_ALLOW = (
        'DeviceTemperature',   # Blackfly/A70/A6701 — primary sensor temp
        'SensorTemperature',   # Blackfly alternate
        'FpaTemperature',      # some FLIR families expose this directly
        'HousingTemperature',  # case/housing
    )
    # Substrings that identify config inputs or manufacturing constants — never
    # publish these as a "camera temperature" measurement.
    TEMP_FEATURE_DENY = (
        'Reflected', 'Atmospheric', 'ExtOptics',
        'Mfg', 'Calibration', 'Correction', 'Delta',
        'Coeff', 'Query', 'MinTemp', 'MaxTemp', 'Flag',
    )

    def _kelvin_to_c(self, val, unit):
        """Normalize a temperature reading to Celsius. FLIR uses Kelvin for the
        atmospheric/scene inputs and Celsius for DeviceTemperature — we only
        publish the latter, but normalize for safety."""
        if unit and 'kelvin' in unit.lower():
            return val - 273.15
        return val

    def _discover_temp_nodes(self, nodemap):
        """Enumerate the GenICam tree and return readable CFloat nodes that
        look like genuine sensor temperature measurements, filtered by
        TEMP_FEATURE_ALLOW / TEMP_FEATURE_DENY. Run once after BeginAcquisition
        to learn what the running firmware actually exposes."""
        found = []
        try:
            root = PySpin.CCategoryPtr(nodemap.GetNode('Root'))
            stack = [root]
            seen = set()
            while stack:
                cat = stack.pop()
                if not PySpin.IsAvailable(cat) or not PySpin.IsReadable(cat):
                    continue
                for feat in cat.GetFeatures():
                    try:
                        name = feat.GetName()
                    except Exception:
                        continue
                    if name in seen:
                        continue
                    seen.add(name)
                    try:
                        iface_type = feat.GetPrincipalInterfaceType()
                    except Exception:
                        iface_type = None
                    if iface_type == PySpin.intfICategory:
                        stack.append(PySpin.CCategoryPtr(feat))
                        continue
                    if 'Temp' not in name:
                        continue
                    if any(bad in name for bad in self.TEMP_FEATURE_DENY):
                        continue
                    if not any(good in name for good in self.TEMP_FEATURE_ALLOW):
                        continue
                    if iface_type == PySpin.intfIFloat:
                        fn = PySpin.CFloatPtr(feat)
                        if PySpin.IsAvailable(fn) and PySpin.IsReadable(fn):
                            try:
                                val = float(fn.GetValue())
                                unit = fn.GetUnit() if hasattr(fn, 'GetUnit') else ''
                                found.append((name, val, unit))
                            except Exception:
                                pass
        except Exception as e:
            self.get_logger().warn(f'_discover_temp_nodes failed: {e}')
        return found

    def _read_and_publish_temps(self):
        """Read temperature features from the camera and publish them. Must be
        called from the acquisition thread so it serializes with GetNextImage —
        PySpin nodemap reads are not thread-safe. On first call after connect,
        enumerate the nodemap to learn which temperature feature names this
        camera firmware actually exposes (varies across A6701/A70/Blackfly).

        Note: A6701 is a cryocooled InSb MWIR camera — its DeviceTemperature
        reports the Stirling-cooled FPA in °C and lives near -196°C (~77K).
        A70 is an uncooled microbolometer — DeviceTemperature is body-ish
        (~30-50°C). Same topic, very different healthy bands: the soak
        monitor must threshold per camera family."""
        if self.cam is None:
            return
        try:
            nodemap = self.cam.GetNodeMap()
        except PySpin.SpinnakerException as e:
            self.get_logger().warn(f'temp read: GetNodeMap failed: {e}')
            return

        if not self._temp_discovery_done:
            found = self._discover_temp_nodes(nodemap)
            if found:
                names = ', '.join(f'{n}={v:.1f}{u or "C"}' for n, v, u in found)
                self.get_logger().info(f'Temperature nodes discovered: {names}')
                self._temp_node_units = {n: u for n, _, u in found}
                # Priority order: sensor/device first, then housing.
                sensor_names = [n for n in self._temp_node_units
                                if any(k in n for k in ('DeviceTemperature',
                                                        'SensorTemperature',
                                                        'FpaTemperature'))]
                housing_names = [n for n in self._temp_node_units
                                 if 'HousingTemperature' in n]
                self._temp_primary = sensor_names[0] if sensor_names else None
                self._temp_housing = housing_names[0] if housing_names else None
            else:
                self.get_logger().warn(
                    'No usable temperature nodes on this camera — expected '
                    'DeviceTemperature/SensorTemperature/HousingTemperature'
                )
                self._temp_node_units = {}
                self._temp_primary = None
                self._temp_housing = None
            self._temp_discovery_done = True

        fpa_c = None
        hsg_c = None
        if self._temp_primary:
            raw = self._read_float_node(nodemap, self._temp_primary)
            if raw is not None:
                fpa_c = self._kelvin_to_c(raw, self._temp_node_units.get(self._temp_primary, ''))
                self._publish_temp_msg(self.fpa_temp_pub, fpa_c)
        if self._temp_housing:
            raw = self._read_float_node(nodemap, self._temp_housing)
            if raw is not None:
                hsg_c = self._kelvin_to_c(raw, self._temp_node_units.get(self._temp_housing, ''))
                self._publish_temp_msg(self.housing_temp_pub, hsg_c)

        parts = []
        if fpa_c is not None:
            parts.append(f'FPA={fpa_c:.1f}C')
        if hsg_c is not None:
            parts.append(f'Housing={hsg_c:.1f}C')
        if parts:
            self.get_logger().info('Camera temps: ' + ' '.join(parts))

    def _release_camera(self):
        """Release current camera handle."""
        if self.cam is not None:
            try:
                self.cam.EndAcquisition()
            except Exception:
                pass
            try:
                self.cam.DeInit()
            except Exception:
                pass
            self.cam = None
        self._temp_discovery_done = False
        self._temp_node_units = {}
        self._temp_primary = None
        self._temp_housing = None
        self._temp_nodes_missing.clear()

    def _reconnect_sleep(self):
        """Sleep before next reconnect, escalating to stuck back-off if the
        camera has gone through stuck_threshold cycles without publishing.
        Counter is reset to 0 the first time we see a successful frame."""
        self._reconnects_without_publish += 1
        if self._reconnects_without_publish >= self.stuck_threshold:
            self.get_logger().warn(
                f'STUCK_BACKOFF: {self._reconnects_without_publish} reconnects '
                f'without a publish line — Pleora session probably stuck, '
                f'sleeping {self.stuck_backoff_s:.0f}s instead of '
                f'{self.reconnect_interval:.0f}s to let firmware drain'
            )
            time.sleep(self.stuck_backoff_s)
        else:
            time.sleep(self.reconnect_interval)

    def _run_loop(self):
        """Main loop: connect, acquire, reconnect on failure."""
        while self.running and rclpy.ok():
            # --- Connect phase ---
            if self.cam is None:
                if not self.connect_camera():
                    self.get_logger().warn(
                        f'Retrying (stuck_count={self._reconnects_without_publish})...'
                    )
                    self._reconnect_sleep()
                    continue

            # --- Acquire phase ---
            try:
                self.cam.BeginAcquisition()
                self.get_logger().info('Started acquisition')
            except PySpin.SpinnakerException as e:
                self.get_logger().error(f'BeginAcquisition failed: {e}')
                self._release_camera()
                self._reconnect_sleep()
                continue

            consecutive_errors = 0
            frame_count = 0
            last_log = time.time()
            # Rate-watchdog state: track the first time the rate-log saw
            # fps < rate_floor_hz. If that persists for rate_floor_window_s
            # we reconnect even without consecutive errors tripping.
            low_rate_since = None
            # Prime the temp topic right away so subscribers don't wait
            # temperature_period_sec for the first reading.
            self._read_and_publish_temps()
            self._last_temp_read = time.time()

            while self.running and rclpy.ok():
                try:
                    image = self.cam.GetNextImage(2000)
                    if image.IsIncomplete():
                        status = image.GetImageStatus()
                        if consecutive_errors < 3:
                            self.get_logger().warn(
                                f'Incomplete frame: status={status}'
                            )
                        image.Release()
                        consecutive_errors += 1
                    else:
                        self._publish_image(image)
                        image.Release()
                        frame_count += 1
                        consecutive_errors = 0

                    now = time.time()
                    if now - last_log >= 5.0:
                        fps = frame_count / (now - last_log)
                        self.get_logger().info(f'Publishing at {fps:.1f} Hz')
                        # First successful Publishing line clears stuck-state.
                        if self._reconnects_without_publish > 0:
                            self.get_logger().info(
                                f'stuck_count cleared after '
                                f'{self._reconnects_without_publish} reconnect(s)'
                            )
                            self._reconnects_without_publish = 0
                        # Rate-watchdog: mark/clear the low-rate-since anchor.
                        if self.rate_floor_hz > 0 and fps < self.rate_floor_hz:
                            if low_rate_since is None:
                                low_rate_since = now
                                self.get_logger().warn(
                                    f'Rate below floor ({fps:.1f} < '
                                    f'{self.rate_floor_hz} Hz), watching...'
                                )
                        else:
                            if low_rate_since is not None:
                                self.get_logger().info(
                                    f'Rate recovered to {fps:.1f} Hz'
                                )
                            low_rate_since = None
                        frame_count = 0
                        last_log = now

                    if self.temp_period > 0 and (now - self._last_temp_read) >= self.temp_period:
                        self._read_and_publish_temps()
                        self._last_temp_read = now

                except PySpin.SpinnakerException as e:
                    consecutive_errors += 1
                    if consecutive_errors <= 3:
                        self.get_logger().warn(f'Acquisition error ({consecutive_errors}): {e}')
                    time.sleep(0.1)

                # If too many consecutive errors, reconnect
                if consecutive_errors >= self.error_threshold:
                    self.get_logger().error(
                        f'{consecutive_errors} consecutive errors, reconnecting...'
                    )
                    break

                # Rate-watchdog: force reconnect if rate stays below floor
                # for the full window. Catches the sustained-degraded state
                # where successful frames keep resetting consecutive_errors.
                if (
                    low_rate_since is not None
                    and (time.time() - low_rate_since) >= self.rate_floor_window_s
                ):
                    self.get_logger().error(
                        f'Rate stayed below {self.rate_floor_hz} Hz for '
                        f'{self.rate_floor_window_s:.0f}s, reconnecting...'
                    )
                    break

            # --- Reconnect phase ---
            self._release_camera()
            if self.running:
                self.get_logger().info(
                    f'Reconnecting (stuck_count={self._reconnects_without_publish})...'
                )
                self._reconnect_sleep()

    def _publish_image(self, image):
        """Convert Spinnaker image to ROS msg and publish."""
        try:
            width = image.GetWidth()
            height = image.GetHeight()
            img = image.GetNDArray()

            if self.strip_metadata and height == 513:
                img = img[1:, :]
                height = 512

            msg = Image()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.height = height
            msg.width = width
            if self.pixel_format == 'Mono8':
                msg.encoding = 'mono8'
                msg.step = width
            else:
                msg.encoding = 'mono16'
                msg.step = width * 2
            msg.is_bigendian = False
            msg.data = img.tobytes()
            self.image_pub.publish(msg)

            if self.publish_info:
                info = CameraInfo()
                info.header = msg.header
                info.height = height
                info.width = width
                self.info_pub.publish(info)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=5.0)
        self._release_camera()
        if self.system is not None:
            try:
                self.system.ReleaseInstance()
            except Exception:
                pass
            self.system = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThermalSpinnakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
