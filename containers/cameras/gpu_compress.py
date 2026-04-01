#!/usr/bin/env python3
"""
GPU-accelerated JPEG compression for ROS2 camera images using NVJPEG.
Subscribes to raw images and publishes compressed versions.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2

# Try to import NVJPEG, fall back to CPU if not available
try:
    from nvjpeg import NvJpeg
    HAVE_NVJPEG = True
except ImportError:
    HAVE_NVJPEG = False

class GpuCompressNode(Node):
    def __init__(self):
        super().__init__('gpu_compress')

        # Parameters
        self.declare_parameter('quality', 85)
        self.declare_parameter('input_topic', '/camera1/image_raw')
        self.declare_parameter('output_topic', '/camera1/image_compressed')

        quality = self.get_parameter('quality').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Initialize encoder
        if HAVE_NVJPEG:
            self.encoder = NvJpeg()
            self.get_logger().info(f'Using NVJPEG GPU compression (quality={quality})')
        else:
            self.encoder = None
            self.get_logger().warn('NVJPEG not available, using CPU compression')

        self.quality = quality

        # QoS for sensor data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber and publisher
        self.sub = self.create_subscription(
            Image, input_topic, self.image_callback, qos)
        self.pub = self.create_publisher(
            CompressedImage, output_topic, qos)

        self.get_logger().info(f'Compressing {input_topic} -> {output_topic}')
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            # Convert ROS Image to numpy array
            if msg.encoding in ['bayer_rggb8', 'bayer_gbrg8', 'bayer_grbg8', 'bayer_bggr8']:
                # Debayer first - map ROS encoding to OpenCV conversion
                bayer_map = {
                    'bayer_rggb8': cv2.COLOR_BAYER_RG2BGR,
                    'bayer_gbrg8': cv2.COLOR_BAYER_GB2BGR,
                    'bayer_grbg8': cv2.COLOR_BAYER_GR2BGR,
                    'bayer_bggr8': cv2.COLOR_BAYER_BG2BGR,
                }
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width)
                img = cv2.cvtColor(img, bayer_map[msg.encoding])
            elif msg.encoding in ['bgr8', 'rgb8']:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'mono8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width)
            else:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
                return

            # Compress
            if HAVE_NVJPEG and self.encoder:
                # GPU compression
                jpeg_data = self.encoder.encode(img, self.quality)
            else:
                # CPU fallback
                _, jpeg_data = cv2.imencode('.jpg', img,
                    [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                jpeg_data = jpeg_data.tobytes()

            # Publish compressed image
            out_msg = CompressedImage()
            out_msg.header = msg.header
            out_msg.format = 'jpeg'
            out_msg.data = jpeg_data
            self.pub.publish(out_msg)

            self.frame_count += 1
            if self.frame_count % 100 == 0:
                ratio = len(jpeg_data) / len(msg.data) * 100
                self.get_logger().info(
                    f'Compressed {self.frame_count} frames, ratio: {ratio:.1f}%')

        except Exception as e:
            self.get_logger().error(f'Compression error: {e}')

def main():
    rclpy.init()
    node = GpuCompressNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
