#!/usr/bin/env python3
"""
FLIR A6701 Thermal Camera ROS2 Node using Spinnaker SDK

The A6701 (Xsc Series) does NOT work with Aravis for streaming -
only Spinnaker can receive frames from this camera.

Publishes raw thermal images as sensor_msgs/Image (Mono16, 640x512).
First row (metadata) is stripped from the 640x513 sensor output.

Usage:
    ros2 run thermal_camera thermal_spinnaker_node --ros-args \
        -r __ns:=/thermal/camera1 \
        -p frame_id:=thermal_camera1_optical_frame
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import numpy as np
import time
import threading
import PySpin


class ThermalSpinnakerNode(Node):
    """ROS2 Node for FLIR A6701 using Spinnaker SDK."""

    def __init__(self):
        super().__init__('thermal_spinnaker_node')

        # Declare parameters
        self.declare_parameter('frame_id', 'thermal_camera')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('strip_metadata_row', True)
        self.declare_parameter('publish_camera_info', True)

        # Get parameters
        self.frame_id = self.get_parameter('frame_id').value
        self.target_fps = self.get_parameter('frame_rate').value
        self.strip_metadata = self.get_parameter('strip_metadata_row').value
        self.publish_info = self.get_parameter('publish_camera_info').value

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', sensor_qos)
        if self.publish_info:
            self.info_pub = self.create_publisher(CameraInfo, 'camera_info', sensor_qos)

        # Camera state
        self.system = None
        self.cam = None
        self.running = False
        self.frame_count = 0
        self.last_log_time = time.time()

        # Connect to camera
        if not self.connect_camera():
            self.get_logger().error('Failed to connect to A6701')
            return

        # Start acquisition thread
        self.running = True
        self.acquisition_thread = threading.Thread(target=self.acquisition_loop)
        self.acquisition_thread.daemon = True
        self.acquisition_thread.start()

        self.get_logger().info(f'A6701 Spinnaker node started: {self.frame_id}')

    def connect_camera(self) -> bool:
        """Connect to the A6701 via Spinnaker."""
        try:
            self.system = PySpin.System.GetInstance()
            cam_list = self.system.GetCameras()
            n_cams = cam_list.GetSize()
            self.get_logger().info(f'Spinnaker found {n_cams} camera(s)')

            # Find A6701 (Xsc Series) - skip BlackFly cameras
            for i in range(n_cams):
                cam = cam_list.GetByIndex(i)
                cam.Init()
                nodemap = cam.GetTLDeviceNodeMap()
                model_node = PySpin.CStringPtr(nodemap.GetNode('DeviceModelName'))
                if model_node.IsValid() and PySpin.IsReadable(model_node):
                    model = model_node.GetValue()
                    self.get_logger().info(f'Camera {i}: {model}')
                    if 'Xsc' in model or 'A6701' in model or 'A67' in model:
                        self.cam = cam
                        self.get_logger().info(f'Selected A6701: {model}')
                        break
                    elif 'Blackfly' not in model and 'BFS' not in model and 'A70' not in model:
                        # Non-BlackFly, non-A70 = might be A6701
                        self.cam = cam
                        self.get_logger().info(f'Selected thermal camera: {model}')
                        break
                cam.DeInit()

            if self.cam is None:
                self.get_logger().error('No A6701 found')
                cam_list.Clear()
                return False

            # Configure
            self.configure_camera()

            # Get dimensions
            nodemap = self.cam.GetNodeMap()
            width_node = PySpin.CIntegerPtr(nodemap.GetNode('Width'))
            height_node = PySpin.CIntegerPtr(nodemap.GetNode('Height'))
            self.width = width_node.GetValue() if width_node.IsValid() else 640
            self.height = height_node.GetValue() if height_node.IsValid() else 513
            self.get_logger().info(f'Resolution: {self.width}x{self.height}')

            if self.strip_metadata and self.height == 513:
                self.get_logger().info('Will strip metadata row (513 -> 512)')

            return True

        except PySpin.SpinnakerException as e:
            self.get_logger().error(f'Spinnaker connection error: {e}')
            return False

    def configure_camera(self):
        """Configure A6701 for Mono16 radiometric output."""
        nodemap = self.cam.GetNodeMap()

        # Set PixelFormat to Mono16
        try:
            pixel_format = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
            if pixel_format.IsValid() and PySpin.IsWritable(pixel_format):
                mono16 = pixel_format.GetEntryByName('Mono16')
                if mono16 is not None and PySpin.IsReadable(mono16):
                    pixel_format.SetIntValue(mono16.GetValue())
                    self.get_logger().info('Set PixelFormat=Mono16')
        except PySpin.SpinnakerException as e:
            self.get_logger().warn(f'Could not set PixelFormat: {e}')

        # Set IRFormat to Radiometric
        try:
            ir_format = PySpin.CEnumerationPtr(nodemap.GetNode('IRFormat'))
            if ir_format.IsValid() and PySpin.IsWritable(ir_format):
                radiometric = ir_format.GetEntryByName('Radiometric')
                if radiometric is not None and PySpin.IsReadable(radiometric):
                    ir_format.SetIntValue(radiometric.GetValue())
                    self.get_logger().info('Set IRFormat=Radiometric')
        except PySpin.SpinnakerException as e:
            self.get_logger().warn(f'Could not set IRFormat: {e}')

        # Log current settings
        try:
            pf = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
            if pf.IsValid() and PySpin.IsReadable(pf):
                self.get_logger().info(
                    f'PixelFormat confirmed: {pf.GetCurrentEntry().GetSymbolic()}')
        except:
            pass

    def acquisition_loop(self):
        """Main acquisition loop running in separate thread."""
        try:
            self.cam.BeginAcquisition()
        except PySpin.SpinnakerException as e:
            self.get_logger().error(f'Failed to start acquisition: {e}')
            return

        self.get_logger().info('Started acquisition')

        while self.running and rclpy.ok():
            try:
                image = self.cam.GetNextImage(2000)  # 2 second timeout

                if image.IsIncomplete():
                    image.Release()
                    continue

                self.process_image(image)
                self.frame_count += 1
                image.Release()

                # Log stats every 5 seconds
                now = time.time()
                if now - self.last_log_time >= 5.0:
                    fps = self.frame_count / (now - self.last_log_time)
                    self.get_logger().info(f'Publishing at {fps:.1f} Hz')
                    self.frame_count = 0
                    self.last_log_time = now

            except PySpin.SpinnakerException as e:
                self.get_logger().error(f'Acquisition error: {e}')
                time.sleep(0.1)

        try:
            self.cam.EndAcquisition()
        except:
            pass
        self.get_logger().info('Stopped acquisition')

    def process_image(self, image):
        """Process Spinnaker image and publish as ROS Image."""
        try:
            width = image.GetWidth()
            height = image.GetHeight()

            # GetNDArray returns numpy array directly
            img = image.GetNDArray()

            # Strip metadata row if configured (A6701 sends 513 rows, row 0 is metadata)
            if self.strip_metadata and height == 513:
                img = img[1:, :]
                height = 512

            # Create ROS Image message
            msg = Image()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            msg.height = height
            msg.width = width
            msg.encoding = 'mono16'
            msg.is_bigendian = False
            msg.step = width * 2
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
        """Clean up on shutdown."""
        self.running = False
        if hasattr(self, 'acquisition_thread'):
            self.acquisition_thread.join(timeout=2.0)
        if self.cam is not None:
            try:
                self.cam.DeInit()
                del self.cam
            except:
                pass
        if self.system is not None:
            try:
                self.system.ReleaseInstance()
            except:
                pass
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
