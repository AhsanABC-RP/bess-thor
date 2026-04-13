#!/usr/bin/env python3
"""
BESS Thermal Pavement Analysis Node
Analyses thermal camera imagery to detect pavement anomalies.

Current mode: statistical anomaly detection on per-patch (32x32) temperature
grids. Flags patches that are >N std devs from frame mean as thermal anomalies
and patches with high gradient magnitude as potential cracks.

Future mode: EfficientNet-B0 classification when fine-tuned weights are
available (swap in via model_path parameter).

Subscribes to FLIR A70 thermal cameras (mono16 radiometric).
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Temperature
from std_msgs.msg import String
from cv_bridge import CvBridge

# Optional: EfficientNet for future fine-tuned classification
try:
    import torch
    import timm
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class ThermalAnalysisNode(Node):
    def __init__(self):
        super().__init__('thermal_analysis_node')

        # Parameters
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('anomaly_threshold', 2.0)
        self.declare_parameter('ambient_temp', 20.0)
        self.declare_parameter('model_path', '')  # Empty = use statistical mode
        self.declare_parameter('patch_size', 32)
        self.declare_parameter('gradient_threshold', 5.0)  # deg C per pixel

        self.device = self.get_parameter('device').value
        self.anomaly_threshold = self.get_parameter('anomaly_threshold').value
        self.ambient_temp = self.get_parameter('ambient_temp').value
        self.model_path = self.get_parameter('model_path').value
        self.patch_size = self.get_parameter('patch_size').value
        self.gradient_threshold = self.get_parameter('gradient_threshold').value

        self.bridge = CvBridge()

        # Load EfficientNet model if weights are provided
        self.model = None
        if self.model_path and TORCH_AVAILABLE:
            self.get_logger().info(
                f'Loading EfficientNet model from {self.model_path} on {self.device}...'
            )
            self.model = timm.create_model(
                'efficientnet_b0', pretrained=False, num_classes=4
            )
            state_dict = torch.load(self.model_path, map_location=self.device)
            self.model.load_state_dict(state_dict)
            self.model.to(self.device)
            self.model.eval()
            if 'cuda' in self.device:
                self.model.half()
            self.get_logger().info('EfficientNet model loaded (FP16)')
        else:
            self.get_logger().info(
                'No model_path provided — using statistical anomaly detection mode'
            )

        # QoS for sensor topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to ambient temperature for radiometric correction
        self.create_subscription(
            Temperature, '/environment/temperature',
            self._on_ambient_temp, 10
        )

        # Subscribe to both FLIR A70 thermal cameras (mono16 radiometric)
        self.thermal_topics = [
            '/thermal1/camera_driver/image_raw',
            '/thermal2/camera_driver/image_raw',
        ]

        self.image_subs = []
        for topic in self.thermal_topics:
            sub = self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self.image_callback(msg, t),
                qos
            )
            self.image_subs.append(sub)
            self.get_logger().info(f'Subscribed: {topic}')

        # Publishers
        self.heatmap_pub = self.create_publisher(
            Image, '/thermal/analysis/heatmap', 10
        )
        self.anomalies_pub = self.create_publisher(
            String, '/thermal/analysis/anomaly_report', 10
        )

        # Stats
        self.frame_count = 0
        self.total_process_time = 0.0
        self.anomaly_count = 0
        self.crack_count = 0
        self.stats_timer = self.create_timer(10.0, self.print_stats)

        self.get_logger().info(
            f'Thermal Analysis Node initialized — '
            f'anomaly_threshold={self.anomaly_threshold} std, '
            f'ambient_temp={self.ambient_temp}C, '
            f'patch_size={self.patch_size}px'
        )

    def _on_ambient_temp(self, msg):
        old = self.ambient_temp
        self.ambient_temp = msg.temperature
        if abs(old - self.ambient_temp) > 0.5:
            self.get_logger().info(
                f'Ambient temp updated: {old:.1f}°C -> {self.ambient_temp:.1f}°C '
                f'(source: {msg.header.frame_id})'
            )

    def raw_to_celsius(self, raw: np.ndarray) -> np.ndarray:
        """
        Convert FLIR A70 mono16 radiometric data to temperature in Celsius.
        Raw radiometric formula for FLIR streaming: T = raw * 0.004 + (ambient - 84.0)
        """
        return raw.astype(np.float32) * 0.004 + (self.ambient_temp - 84.0)

    def analyse_statistical(self, temp_c: np.ndarray, stamp_sec: float):
        """
        Statistical anomaly detection on a 32x32 patch grid.
        Returns list of anomaly dicts and a coloured heatmap.
        """
        h, w = temp_c.shape
        ps = self.patch_size
        anomalies = []

        # Frame-level statistics
        frame_mean = np.mean(temp_c)
        frame_std = np.std(temp_c)

        if frame_std < 0.01:
            # Uniform frame, nothing to detect
            frame_std = 0.01

        # Compute gradient magnitude for crack detection
        grad_x = cv2.Sobel(temp_c, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(temp_c, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(grad_x ** 2 + grad_y ** 2)

        # Iterate over patches
        for py in range(0, h - ps + 1, ps):
            for px in range(0, w - ps + 1, ps):
                patch_temp = temp_c[py:py + ps, px:px + ps]
                patch_grad = grad_mag[py:py + ps, px:px + ps]

                patch_mean = np.mean(patch_temp)
                patch_grad_max = np.max(patch_grad)

                temp_delta = patch_mean - frame_mean

                # Thermal anomaly: patch mean deviates significantly from frame
                if abs(temp_delta) > self.anomaly_threshold * frame_std:
                    anomalies.append({
                        'class': 'thermal_anomaly',
                        'confidence': round(
                            min(abs(temp_delta) / (frame_std * 4.0), 1.0), 3
                        ),
                        'x': px,
                        'y': py,
                        'temp_delta': round(float(temp_delta), 2),
                        'patch_size': ps,
                        'timestamp': round(stamp_sec, 6),
                    })
                    self.anomaly_count += 1

                # Potential crack: high thermal gradient within patch
                if patch_grad_max > self.gradient_threshold:
                    anomalies.append({
                        'class': 'potential_crack',
                        'confidence': round(
                            min(patch_grad_max / (self.gradient_threshold * 3.0), 1.0), 3
                        ),
                        'x': px,
                        'y': py,
                        'temp_delta': round(float(temp_delta), 2),
                        'patch_size': ps,
                        'timestamp': round(stamp_sec, 6),
                    })
                    self.crack_count += 1

        # Generate coloured heatmap
        heatmap = self._create_heatmap(temp_c, frame_mean, frame_std)

        return anomalies, heatmap

    def _create_heatmap(
        self, temp_c: np.ndarray, frame_mean: float, frame_std: float
    ) -> np.ndarray:
        """
        Create a BGR8 coloured temperature overlay using COLORMAP_INFERNO.
        Normalises around frame mean +/- 3 std for good contrast.
        """
        vmin = frame_mean - 3.0 * frame_std
        vmax = frame_mean + 3.0 * frame_std
        if vmax <= vmin:
            vmax = vmin + 1.0

        normalised = np.clip((temp_c - vmin) / (vmax - vmin), 0.0, 1.0)
        normalised_u8 = (normalised * 255).astype(np.uint8)
        heatmap = cv2.applyColorMap(normalised_u8, cv2.COLORMAP_INFERNO)

        # Overlay temperature scale text
        cv2.putText(
            heatmap,
            f'{vmin:.1f}C',
            (5, heatmap.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )
        cv2.putText(
            heatmap,
            f'{vmax:.1f}C',
            (5, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )
        cv2.putText(
            heatmap,
            f'mean={frame_mean:.1f}C',
            (heatmap.shape[1] - 160, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )

        return heatmap

    def image_callback(self, msg: Image, topic: str):
        """Process incoming thermal image."""
        try:
            # FLIR A70 streams mono16 radiometric
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')

            start_time = time.time()

            # Convert raw radiometric to Celsius
            temp_c = self.raw_to_celsius(cv_image)

            # Timestamp from message header
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # Run analysis
            if self.model is not None:
                # Future: EfficientNet classification path
                anomalies, heatmap = self._analyse_with_model(temp_c, stamp_sec)
            else:
                anomalies, heatmap = self.analyse_statistical(temp_c, stamp_sec)

            process_time = time.time() - start_time
            self.total_process_time += process_time
            self.frame_count += 1

            # Publish heatmap as bgr8 Image
            heatmap_msg = self.bridge.cv2_to_imgmsg(heatmap, encoding='bgr8')
            heatmap_msg.header = msg.header
            self.heatmap_pub.publish(heatmap_msg)

            # Publish anomalies as JSON
            anom_msg = String()
            anom_msg.data = json.dumps(anomalies)
            self.anomalies_pub.publish(anom_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing thermal image from {topic}: {e}')

    def _analyse_with_model(self, temp_c: np.ndarray, stamp_sec: float):
        """
        EfficientNet-based classification path.
        Splits the thermal image into patches, classifies each with the model.
        Placeholder until fine-tuned weights are available.
        """
        h, w = temp_c.shape
        ps = self.patch_size
        anomalies = []

        frame_mean = np.mean(temp_c)
        frame_std = np.std(temp_c)

        # Normalise to 0-255 for model input
        vmin = frame_mean - 3.0 * max(frame_std, 0.01)
        vmax = frame_mean + 3.0 * max(frame_std, 0.01)
        norm = np.clip((temp_c - vmin) / (vmax - vmin), 0.0, 1.0)
        norm_u8 = (norm * 255).astype(np.uint8)

        # Convert to 3-channel for EfficientNet
        rgb = cv2.cvtColor(norm_u8, cv2.COLOR_GRAY2RGB)

        # Class labels for the fine-tuned model
        class_names = ['normal', 'thermal_anomaly', 'potential_crack', 'subsurface_void']

        for py in range(0, h - ps + 1, ps):
            for px in range(0, w - ps + 1, ps):
                patch = rgb[py:py + ps, px:px + ps]

                # Resize to EfficientNet input size
                patch_resized = cv2.resize(patch, (224, 224))
                tensor = torch.from_numpy(patch_resized).permute(2, 0, 1).float()
                tensor = tensor.unsqueeze(0) / 255.0
                tensor = tensor.to(self.device)
                if 'cuda' in self.device:
                    tensor = tensor.half()

                with torch.no_grad():
                    logits = self.model(tensor)
                    probs = torch.softmax(logits, dim=1)
                    conf, cls_idx = torch.max(probs, dim=1)
                    conf = float(conf[0])
                    cls_idx = int(cls_idx[0])

                cls_name = class_names[cls_idx]
                if cls_name != 'normal' and conf > 0.5:
                    patch_temp = temp_c[py:py + ps, px:px + ps]
                    temp_delta = float(np.mean(patch_temp) - frame_mean)

                    anomalies.append({
                        'class': cls_name,
                        'confidence': round(conf, 3),
                        'x': px,
                        'y': py,
                        'temp_delta': round(temp_delta, 2),
                        'patch_size': ps,
                        'timestamp': round(stamp_sec, 6),
                    })

                    if cls_name == 'thermal_anomaly':
                        self.anomaly_count += 1
                    elif cls_name == 'potential_crack':
                        self.crack_count += 1

        heatmap = self._create_heatmap(temp_c, frame_mean, frame_std)
        return anomalies, heatmap

    def print_stats(self):
        """Print performance statistics every 10 seconds."""
        if self.frame_count > 0:
            avg_time = self.total_process_time / self.frame_count
            fps = 1.0 / avg_time if avg_time > 0 else 0
            mode = 'model' if self.model is not None else 'statistical'
            self.get_logger().info(
                f'Stats: {self.frame_count} frames, '
                f'avg process: {avg_time*1000:.1f}ms, '
                f'throughput: {fps:.1f} fps, '
                f'anomalies: {self.anomaly_count}, '
                f'cracks: {self.crack_count}, '
                f'mode: {mode}'
            )
            self.frame_count = 0
            self.total_process_time = 0.0
            self.anomaly_count = 0
            self.crack_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = ThermalAnalysisNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
