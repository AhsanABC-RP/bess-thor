#!/usr/bin/env python3
"""
Lucid Vision Atlas Camera ROS2 Node

Simple Python node for Lucid Atlas 64MP cameras using Arena SDK.
Alternative to the official arena_camera_node if it doesn't work on ARM64/Jazzy.

Usage:
    python3 lucid_node.py --ros-args -p camera_ip:=169.254.x.x -p frame_id:=camera1_optical_frame

Environment:
    ARENA_ROOT: Path to Arena SDK installation
    CAMERA_IP: Camera IP address (optional, auto-discover if not set)
    CAMERA_NAME: Camera name for topic namespace
    FRAME_ID: TF frame ID for image messages
"""

import os
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

# Arena SDK import
try:
    from arena_api.system import system
    from arena_api.buffer import BufferFactory
    ARENA_AVAILABLE = True
except ImportError:
    ARENA_AVAILABLE = False
    print("WARNING: arena_api not found. Install from Arena SDK.")


class LucidCameraNode(Node):
    def __init__(self):
        super().__init__('lucid_camera_node')

        # Parameters
        self.declare_parameter('camera_ip', os.environ.get('CAMERA_IP', ''))
        self.declare_parameter('camera_mac', os.environ.get('CAMERA_MAC', ''))
        self.declare_parameter('camera_name', os.environ.get('CAMERA_NAME', 'camera1'))
        self.declare_parameter('frame_id', os.environ.get('FRAME_ID', 'camera1_optical_frame'))
        self.declare_parameter('exposure_us', 10000.0)
        self.declare_parameter('gain_db', 0.0)
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('binning', 1)
        self.declare_parameter('pixel_format', 'BayerGB8')  # ATX650G-C default

        self.camera_ip = self.get_parameter('camera_ip').value
        self.camera_mac = self.get_parameter('camera_mac').value.lower().strip()
        self.camera_name = self.get_parameter('camera_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.exposure_us = self.get_parameter('exposure_us').value
        self.gain_db = self.get_parameter('gain_db').value
        self.target_fps = self.get_parameter('fps').value
        self.binning = self.get_parameter('binning').value
        self.pixel_format = self.get_parameter('pixel_format').value

        # QoS for sensor data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        ns = f'/{self.camera_name}/camera_driver'
        self.image_pub = self.create_publisher(Image, f'{ns}/image_raw', qos)
        self.info_pub = self.create_publisher(CameraInfo, f'{ns}/camera_info', qos)

        self.device = None
        self.stream = None
        self.running = True
        self.frame_count = 0
        self.last_log_time = time.time()

        self.get_logger().info(f"Lucid Camera Node starting...")
        self.get_logger().info(f"  Camera IP: {self.camera_ip or 'auto-discover'}")
        self.get_logger().info(f"  Namespace: {ns}")
        self.get_logger().info(f"  Frame ID: {self.frame_id}")

        # Connect to camera. Fail loudly so the process exits non-zero and the
        # docker-compose bash while-true loop respawns us instead of leaving a
        # ghost-alive python3 spinning forever in rclpy.spin() with no work.
        if not self.connect_camera():
            self.get_logger().error("Failed to connect to camera. Exiting.")
            raise RuntimeError("lucid camera connect failed")

        # Start acquisition timer
        period = 1.0 / self.target_fps
        self.timer = self.create_timer(period, self.acquisition_callback)

        # Live exposure/gain tweak during calibration — honour ros2 param set
        self.add_on_set_parameters_callback(self._on_param_set)

    def _on_param_set(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'exposure_us':
                self.exposure_us = float(p.value)
                self._apply_exposure()
            elif p.name == 'gain_db':
                self.gain_db = float(p.value)
                self._apply_gain()
        return SetParametersResult(successful=True)

    def _apply_exposure(self):
        if self.device is None:
            return
        try:
            nodemap = self.device.nodemap
            if self.exposure_us > 0:
                nodemap['ExposureAuto'].value = 'Off'
                nodemap['ExposureTime'].value = float(self.exposure_us)
                self.get_logger().info(f"Live exposure: {self.exposure_us}us")
            else:
                nodemap['ExposureAuto'].value = 'Continuous'
                self.get_logger().info("Live exposure: auto")
        except Exception as e:
            self.get_logger().warning(f"Live exposure set failed: {e}")

    def _apply_gain(self):
        if self.device is None:
            return
        try:
            nodemap = self.device.nodemap
            if self.gain_db > 0:
                nodemap['GainAuto'].value = 'Off'
                nodemap['Gain'].value = float(self.gain_db)
                self.get_logger().info(f"Live gain: {self.gain_db}dB")
            else:
                # Arena SDK on some Lucid models rejects writing 'Continuous' to GainAuto
                # if GainAutoEnable is gated elsewhere — probe for the enum entry first.
                try:
                    entries = [e.symbolic for e in nodemap['GainAuto'].enumentries]
                except Exception:
                    entries = ['Continuous']
                target = 'Continuous' if 'Continuous' in entries else ('Once' if 'Once' in entries else entries[0])
                nodemap['GainAuto'].value = target
                self.get_logger().info(f"Live gain: auto ({target})")
        except Exception as e:
            self.get_logger().warning(f"Live gain set failed: {type(e).__name__}: {e!r}")

    def connect_camera(self):
        """Connect to Lucid camera via Arena SDK."""
        if not ARENA_AVAILABLE:
            self.get_logger().error("Arena SDK not available")
            return False

        try:
            # Discover cameras - ONLY open Lucid cameras, not all GigE devices
            system.destroy_device()

            # Get device info list first (non-exclusive enumeration)
            device_infos = system.device_infos
            self.get_logger().info(f"Found {len(device_infos)} GigE device(s) total")

            # Filter for Lucid cameras only
            lucid_infos = [d for d in device_infos if 'Lucid' in d.get('vendor', '')]
            self.get_logger().info(f"Found {len(lucid_infos)} Lucid camera(s)")

            if not lucid_infos:
                self.get_logger().error("No Lucid cameras found")
                return False

            # Further filter by MAC (preferred — robust against auto-IP changes)
            # or IP (fallback for legacy configs).
            if self.camera_mac:
                def _mac_norm(m):
                    return (m or '').lower().replace('-', ':').replace('.', ':').strip()
                target = _mac_norm(self.camera_mac)
                lucid_infos = [d for d in lucid_infos if _mac_norm(d.get('mac', '')) == target]
                if not lucid_infos:
                    self.get_logger().error(f"No Lucid camera found with MAC: {self.camera_mac}")
                    return False
                self.get_logger().info(f"Matched Lucid by MAC: {self.camera_mac}")
            elif self.camera_ip:
                lucid_infos = [d for d in lucid_infos if d.get('ip') == self.camera_ip]
                if not lucid_infos:
                    self.get_logger().error(f"No Lucid camera found at IP: {self.camera_ip}")
                    return False

            # Open ONLY the filtered Lucid camera(s)
            devices = system.create_device(device_infos=lucid_infos[:1])

            if not devices:
                self.get_logger().error("Failed to open Lucid camera")
                return False

            # Use the first (and only) device we opened
            self.device = devices[0]
            nodemap = self.device.nodemap
            self.nodemap = nodemap
            model = nodemap['DeviceModelName'].value
            serial = nodemap['DeviceSerialNumber'].value
            dev_ip = nodemap['GevCurrentIPAddress'].value
            ip_str = f"{(dev_ip>>24)&0xFF}.{(dev_ip>>16)&0xFF}.{(dev_ip>>8)&0xFF}.{dev_ip&0xFF}"

            self.get_logger().info(f"Connected: {model} (SN: {serial}) @ {ip_str}")

            # Configure camera

            # Set pixel format
            try:
                nodemap['PixelFormat'].value = self.pixel_format
            except Exception as e:
                self.get_logger().warning(f"Could not set pixel format: {e}")

            # Set binning
            if self.binning > 1:
                try:
                    nodemap['BinningHorizontal'].value = self.binning
                    nodemap['BinningVertical'].value = self.binning
                except Exception:
                    pass

            # Set exposure — use Continuous auto if exposure_us <= 0
            try:
                if self.exposure_us > 0:
                    nodemap['ExposureAuto'].value = 'Off'
                    nodemap['ExposureTime'].value = self.exposure_us
                    self.get_logger().info(f"Exposure: {self.exposure_us}us (manual)")
                else:
                    nodemap['ExposureAuto'].value = 'Continuous'
                    self.get_logger().info("Exposure: auto (continuous)")
            except Exception as e:
                self.get_logger().warning(f"Could not set exposure: {e}")

            # Set gain
            try:
                if self.gain_db > 0:
                    nodemap['GainAuto'].value = 'Off'
                    nodemap['Gain'].value = self.gain_db
                    self.get_logger().info(f"Gain: {self.gain_db}dB")
                else:
                    nodemap['GainAuto'].value = 'Continuous'
                    self.get_logger().info("Gain: auto (continuous)")
            except Exception as e:
                self.get_logger().warning(f"Could not set gain: {e}")

            # Enable frame rate control
            try:
                nodemap['AcquisitionFrameRateEnable'].value = True
                nodemap['AcquisitionFrameRate'].value = self.target_fps
            except Exception:
                pass

            # Configure GigE Vision streaming
            try:
                # Packet size must be < MTU (1466 on mgbe interfaces)
                nodemap['GevSCPSPacketSize'].value = 1400
                self.get_logger().info("Set packet size to 1400")
            except Exception as e:
                self.get_logger().warning(f"Could not set packet size: {e}")

            try:
                # Inter-packet delay - give some breathing room
                nodemap['GevSCPD'].value = 1000  # 1000 ticks delay
            except Exception:
                pass

            try:
                # Stream channel packet destination
                nodemap['StreamAutoNegotiatePacketSize'].value = True
            except Exception:
                pass

            # Enable PTP clock sync (IEEE 1588) — Thor mgbe0_0 is grandmaster.
            # Per Lucid KB "PTP with LUCID cameras", set PtpEnable=True and
            # PtpSlaveOnly=True so the camera never tries to become master and
            # always follows our ptp4l-mgbe0 instance. PtpDataSetLatch then
            # snapshots the internal PTP state for reading via PtpStatus.
            try:
                nodemap['PtpEnable'].value = True
                self.get_logger().info("PtpEnable=True")
                try:
                    nodemap['PtpSlaveOnly'].value = True
                    self.get_logger().info("PtpSlaveOnly=True")
                except Exception as e:
                    self.get_logger().warning(f"PtpSlaveOnly unavailable: {e}")
                try:
                    nodemap['PtpDataSetLatch'].execute()
                    status = nodemap['PtpStatus'].value
                    self.get_logger().info(f"PtpStatus={status}")
                except Exception as e:
                    self.get_logger().warning(f"PtpStatus read failed: {e}")
            except Exception as e:
                self.get_logger().warning(f"Could not enable PTP: {e}")

            # Get image dimensions
            self.width = nodemap['Width'].value
            self.height = nodemap['Height'].value
            self.get_logger().info(f"Image size: {self.width}x{self.height}")

            # Start stream with increased buffer count for large images
            try:
                self.device.start_stream(5)  # 5 buffers
                self.get_logger().info("Stream started (5 buffers)")
            except Exception as e:
                self.get_logger().warning(f"Stream start with buffers failed: {e}, trying default")
                self.device.start_stream()
                self.get_logger().info("Stream started (default buffers)")

            return True

        except Exception as e:
            self.get_logger().error(f"Camera connection error: {e}")
            return False

    def acquisition_callback(self):
        """Acquire and publish image."""
        if not self.device:
            return

        try:
            # Get buffer with increased timeout for 65MP images
            buffer = self.device.get_buffer(timeout=5000)

            if buffer:
                self.publish_image(buffer)
                self.frame_count += 1

                # Requeue buffer
                self.device.requeue_buffer(buffer)

                # Log rate periodically
                now = time.time()
                if now - self.last_log_time >= 5.0:
                    fps = self.frame_count / (now - self.last_log_time)
                    ptp_status = "?"
                    try:
                        self.nodemap['PtpDataSetLatch'].execute()
                        ptp_status = self.nodemap['PtpStatus'].value
                    except Exception as e:
                        ptp_status = f"latch_err:{e}"
                    self.get_logger().info(f"Publishing at {fps:.1f} Hz PtpStatus={ptp_status}")
                    self.frame_count = 0
                    self.last_log_time = now

        except Exception as e:
            self.get_logger().warning(f"Acquisition error: {e}")

    def publish_image(self, buffer):
        """Convert buffer to ROS Image and publish."""
        try:
            # Create header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.frame_id

            # Get image data from Arena SDK buffer
            width = buffer.width
            height = buffer.height
            bpp = max(1, int(buffer.bits_per_pixel / 8))

            # Determine encoding
            pixel_format = buffer.pixel_format.name
            if 'Mono8' in pixel_format:
                encoding = 'mono8'
                step = width
            elif 'Mono16' in pixel_format:
                encoding = 'mono16'
                step = width * 2
            elif 'RGB8' in pixel_format:
                encoding = 'rgb8'
                step = width * 3
            elif 'BGR8' in pixel_format:
                encoding = 'bgr8'
                step = width * 3
            elif 'BayerRG8' in pixel_format:
                encoding = 'bayer_rggb8'
                step = width
            elif 'BayerBG8' in pixel_format:
                encoding = 'bayer_bggr8'
                step = width
            elif 'BayerGB8' in pixel_format:
                encoding = 'bayer_gbrg8'
                step = width
            elif 'BayerGR8' in pixel_format:
                encoding = 'bayer_grbg8'
                step = width
            else:
                encoding = 'mono8'
                step = width

            # Create Image message
            img_msg = Image()
            img_msg.header = header
            img_msg.height = height
            img_msg.width = width
            img_msg.encoding = encoding
            img_msg.is_bigendian = False
            img_msg.step = step
            # Use np.ctypeslib.as_array to read pixel data from ctypes pointer
            # (bytes(pdata) only returns the 8-byte pointer value on 64-bit)
            nparray = np.ctypeslib.as_array(buffer.pdata, shape=(height * width * bpp,))
            img_msg.data = nparray.tobytes()

            self.image_pub.publish(img_msg)

            # Publish CameraInfo (placeholder - needs calibration)
            info_msg = CameraInfo()
            info_msg.header = header
            info_msg.height = height
            info_msg.width = width
            self.info_pub.publish(info_msg)

        except Exception as e:
            self.get_logger().warning(f"Publish error: {e}")

    def destroy_node(self):
        """Clean up on shutdown."""
        self.running = False
        if self.device:
            try:
                self.device.stop_stream()
                system.destroy_device()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    if not ARENA_AVAILABLE:
        print("ERROR: Arena SDK not installed. Download from https://thinklucid.com/downloads-hub/")
        sys.exit(1)

    rclpy.init(args=args)
    node = None
    try:
        node = LucidCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"ERROR: lucid_node fatal: {e}", flush=True)
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
