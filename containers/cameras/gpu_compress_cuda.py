#!/usr/bin/env python3
"""
GPU-accelerated JPEG compression for ROS2 camera images using NVIDIA nvJPEG.

HARD-FAILS at startup if libnvjpeg cannot be loaded or probe-encode fails.
No silent CPU fallback — silent fallback was the bug we're fixing.

Uses the self-contained ctypes wrapper in nvjpeg_encoder.py so we do not
depend on PyTorch or torchvision wheels (which are CPU-only on aarch64
from pytorch.org's cu124 index).
"""

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nvjpeg_encoder import NvJpegEncoder, NVJPEG_INPUT_RGBI, NVJPEG_INPUT_BGRI

BAYER_MAP = {
    'bayer_rggb8': cv2.COLOR_BAYER_BG2RGB,
    'bayer_gbrg8': cv2.COLOR_BAYER_GR2RGB,
    'bayer_grbg8': cv2.COLOR_BAYER_GB2RGB,
    'bayer_bggr8': cv2.COLOR_BAYER_RG2RGB,
}


class GpuCompressNode(Node):
    def __init__(self):
        super().__init__('gpu_compress')

        self.declare_parameter('quality', 85)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_raw/compressed')

        self.quality = int(self.get_parameter('quality').value)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Initialize GPU encoder (hard-fails at import/init)
        self.encoder = NvJpegEncoder(quality=self.quality)
        self.get_logger().info(
            f"GPU compression active: nvJPEG via ctypes, quality={self.quality}"
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Image, input_topic, self.image_callback, qos
        )
        self.pub = self.create_publisher(CompressedImage, output_topic, qos)

        self.get_logger().info(f'Compressing {input_topic} -> {output_topic}')
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            if msg.encoding in BAYER_MAP:
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width
                )
                rgb = cv2.cvtColor(raw, BAYER_MAP[msg.encoding])
                input_format = NVJPEG_INPUT_RGBI
            elif msg.encoding == 'rgb8':
                rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                input_format = NVJPEG_INPUT_RGBI
            elif msg.encoding == 'bgr8':
                rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                input_format = NVJPEG_INPUT_BGRI
            elif msg.encoding == 'mono8':
                mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width
                )
                rgb = np.stack([mono, mono, mono], axis=-1)
                input_format = NVJPEG_INPUT_RGBI
            elif msg.encoding == 'mono16':
                mono16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    msg.height, msg.width
                )
                mono8 = (mono16 >> 8).astype(np.uint8)
                rgb = np.stack([mono8, mono8, mono8], axis=-1)
                input_format = NVJPEG_INPUT_RGBI
            else:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
                return

            if not rgb.flags['C_CONTIGUOUS']:
                rgb = np.ascontiguousarray(rgb)

            jpeg_bytes = self.encoder.encode(
                rgb.tobytes(), msg.width, msg.height, input_format=input_format
            )

            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = jpeg_bytes
            self.pub.publish(out)

            self.frame_count += 1
            if self.frame_count % 100 == 0:
                ratio = len(jpeg_bytes) / max(1, len(msg.data)) * 100
                self.get_logger().info(
                    f'Compressed {self.frame_count} frames, ratio: {ratio:.1f}%'
                )

        except Exception as exc:
            self.get_logger().error(f'Compression error: {exc}')


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
