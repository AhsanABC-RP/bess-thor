#!/usr/bin/env python3
"""
BESS SegFormer Semantic Segmentation Node
Runs SegFormer-B2 (Cityscapes 19-class) on masked camera images.
Publishes per-pixel class label maps and colored overlay visualizations.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor
import time


# Cityscapes 19-class definitions
CITYSCAPES_CLASSES = [
    'road', 'sidewalk', 'building', 'wall', 'fence', 'pole',
    'traffic_light', 'traffic_sign', 'vegetation', 'terrain',
    'sky', 'person', 'rider', 'car', 'truck', 'bus', 'train',
    'motorcycle', 'bicycle'
]

CITYSCAPES_COLORS = [
    (128, 64, 128),   # road
    (244, 35, 232),   # sidewalk
    (70, 70, 70),     # building
    (102, 102, 156),  # wall
    (190, 153, 153),  # fence
    (153, 153, 153),  # pole
    (250, 170, 30),   # traffic_light
    (220, 220, 0),    # traffic_sign
    (107, 142, 35),   # vegetation
    (152, 251, 152),  # terrain
    (70, 130, 180),   # sky
    (220, 20, 60),    # person
    (255, 0, 0),      # rider
    (0, 0, 142),      # car
    (0, 0, 70),       # truck
    (0, 60, 100),     # bus
    (0, 80, 100),     # train
    (0, 0, 230),      # motorcycle
    (119, 11, 32)     # bicycle
]


class SegFormerNode(Node):
    def __init__(self):
        super().__init__('segformer_node')

        # Parameters
        self.declare_parameter(
            'model_name', 'nvidia/segformer-b2-finetuned-cityscapes-1024-1024'
        )
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('input_size', 1024)
        # Default to Thor's PII-masked compressed topics (from pii_mask_node)
        self.declare_parameter(
            'camera_topics',
            [
                '/blackfly/camera1/blackfly_camera/image_masked/compressed',
                '/blackfly/camera2/blackfly_camera/image_masked/compressed',
                '/lucid1/camera_driver/image_masked/compressed',
                '/lucid2/camera_driver/image_masked/compressed',
            ],
        )

        model_name = self.get_parameter('model_name').value
        self.device = self.get_parameter('device').value
        self.input_size = self.get_parameter('input_size').value

        self.get_logger().info(
            f'Loading SegFormer model "{model_name}" on {self.device}...'
        )

        # Load model and processor
        self.processor = SegformerImageProcessor.from_pretrained(model_name)
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_name)
        self.model.to(self.device)
        self.model.eval()

        # Build color lookup table (BGR for OpenCV)
        self.color_lut = np.zeros((len(CITYSCAPES_COLORS), 3), dtype=np.uint8)
        for i, (r, g, b) in enumerate(CITYSCAPES_COLORS):
            self.color_lut[i] = [b, g, r]  # RGB -> BGR

        # Warmup inference
        dummy = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
        self._infer(dummy)
        self.get_logger().info('Model loaded and warmed up')

        self.bridge = CvBridge()

        # QoS: best-effort, depth 1 — drop frames if processing is slow
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to masked camera images (CompressedImage from PII node)
        self.camera_topics = list(self.get_parameter('camera_topics').value)

        self.label_publishers = {}
        self.colored_publishers = {}
        self.image_subscribers = []

        for topic in self.camera_topics:
            parts = [p for p in topic.strip('/').split('/') if p]
            cam_name = parts[0]

            label_topic = f'/{cam_name}/segmentation/labels'
            colored_topic = f'/{cam_name}/segmentation/colored'

            sub = self.create_subscription(
                CompressedImage, topic,
                lambda msg, t=topic: self.compressed_callback(msg, t),
                qos
            )
            self.image_subscribers.append(sub)

            self.label_publishers[topic] = self.create_publisher(
                Image, label_topic, 10
            )
            self.colored_publishers[topic] = self.create_publisher(
                CompressedImage, colored_topic, 10
            )

            self.get_logger().info(
                f'Subscribed: {topic} -> {label_topic}, {colored_topic}'
            )

        # Per-camera stats
        self.stats = {t: {'count': 0, 'total_time': 0.0} for t in self.camera_topics}
        self.stats_timer = self.create_timer(10.0, self.print_stats)

        self.get_logger().info('SegFormer Node initialized')

    def _infer(self, cv_image):
        """Run SegFormer inference on a BGR OpenCV image.

        Returns:
            label_map: np.ndarray of shape (H, W), dtype uint8 — per-pixel class IDs
        """
        # Convert BGR -> RGB for the model
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Preprocess
        inputs = self.processor(images=rgb, return_tensors='pt')
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        # Inference (no gradient computation)
        with torch.no_grad():
            outputs = self.model(**inputs)

        # Upsample logits to original image size
        logits = outputs.logits  # (1, num_classes, H/4, W/4)
        upsampled = torch.nn.functional.interpolate(
            logits,
            size=(cv_image.shape[0], cv_image.shape[1]),
            mode='bilinear',
            align_corners=False,
        )

        # Argmax to get per-pixel class IDs
        label_map = upsampled.argmax(dim=1).squeeze(0).cpu().numpy().astype(np.uint8)
        return label_map

    def compressed_callback(self, msg, topic):
        """Process incoming masked compressed image through SegFormer."""
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                self.get_logger().warn(f'Failed to decode compressed {topic}')
                return

            start_time = time.time()
            label_map = self._infer(cv_image)
            inference_time = time.time() - start_time

            # Update stats
            self.stats[topic]['count'] += 1
            self.stats[topic]['total_time'] += inference_time

            # Publish label map (mono8, class IDs 0-18)
            label_msg = self.bridge.cv2_to_imgmsg(label_map, encoding='mono8')
            label_msg.header = msg.header
            self.label_publishers[topic].publish(label_msg)

            # Publish colored overlay (compressed JPEG)
            colored = self.color_lut[label_map]  # (H, W, 3) BGR
            # Blend: 60% original + 40% segmentation color
            blended = cv2.addWeighted(cv_image, 0.6, colored, 0.4, 0)

            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = cv2.imencode(
                '.jpg', blended, [cv2.IMWRITE_JPEG_QUALITY, 85]
            )[1].tobytes()
            self.colored_publishers[topic].publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing {topic}: {e}')

    def print_stats(self):
        """Log FPS statistics per camera every 10 seconds."""
        for topic, s in self.stats.items():
            if s['count'] > 0:
                avg_ms = (s['total_time'] / s['count']) * 1000.0
                fps = s['count'] / s['total_time'] if s['total_time'] > 0 else 0
                cam = topic.strip('/').split('/')[0]
                self.get_logger().info(
                    f'[{cam}] {s["count"]} frames, '
                    f'avg: {avg_ms:.1f}ms, '
                    f'{fps:.1f} fps'
                )
                s['count'] = 0
                s['total_time'] = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = SegFormerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
