#!/usr/bin/env python3
"""Thermal colormap overlay node.

Subscribes to raw Mono16 thermal images, applies adaptive percentile
normalization followed by an OpenCV colormap (default: inferno), and
republishes as BGR8 for visualization in Foxglove / Rerun.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

COLORMAP_LOOKUP = {
    "inferno": cv2.COLORMAP_INFERNO,
    "jet": cv2.COLORMAP_JET,
    "magma": cv2.COLORMAP_MAGMA,
    "plasma": cv2.COLORMAP_PLASMA,
    "viridis": cv2.COLORMAP_VIRIDIS,
    "turbo": cv2.COLORMAP_TURBO,
    "hot": cv2.COLORMAP_HOT,
    "bone": cv2.COLORMAP_BONE,
}


class ThermalColormapNode(Node):
    def __init__(self):
        super().__init__("thermal_colormap")

        # Parameters
        self.declare_parameter("colormap", "inferno")
        self.declare_parameter("normalize_percentile_low", 2.0)
        self.declare_parameter("normalize_percentile_high", 98.0)
        self.declare_parameter("roi_fraction", 0.4)  # centre box for normalization (0.4 = middle 40%)

        cmap_name = self.get_parameter("colormap").value
        self.colormap = COLORMAP_LOOKUP.get(cmap_name, cv2.COLORMAP_INFERNO)
        self.pct_low = self.get_parameter("normalize_percentile_low").value
        self.pct_high = self.get_parameter("normalize_percentile_high").value
        self.roi_fraction = self.get_parameter("roi_fraction").value

        self.bridge = CvBridge()

        # Best-effort QoS to match sensor publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        cameras = [
            ("/thermal/camera1/image_raw", "/thermal/camera1/image_color"),
            ("/thermal/camera2/image_raw", "/thermal/camera2/image_color"),
            ("/thermal/camera3/image_raw", "/thermal/camera3/image_color"),
            ("/thermal/camera4/image_raw", "/thermal/camera4/image_color"),
        ]

        self._pubs = {}
        for sub_topic, pub_topic in cameras:
            pub = self.create_publisher(Image, pub_topic, qos)
            self._pubs[sub_topic] = pub
            self.create_subscription(
                Image,
                sub_topic,
                lambda msg, p=pub: self._on_image(msg, p),
                qos,
            )
            self.get_logger().info(f"{sub_topic} -> {pub_topic}")

    def _on_image(self, msg: Image, pub):
        try:
            mono16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}", throttle_duration_sec=5.0)
            return

        # Centre-weighted ROI normalization — avoids sky/ground blowout
        h, w = mono16.shape[:2]
        rf = self.roi_fraction
        y1, y2 = int(h * (0.5 - rf / 2)), int(h * (0.5 + rf / 2))
        x1, x2 = int(w * (0.5 - rf / 2)), int(w * (0.5 + rf / 2))
        roi = mono16[y1:y2, x1:x2]
        lo = np.percentile(roi, self.pct_low)
        hi = np.percentile(roi, self.pct_high)
        if hi <= lo:
            hi = lo + 1.0

        normalized = np.clip((mono16.astype(np.float32) - lo) / (hi - lo) * 255.0, 0, 255).astype(
            np.uint8
        )

        # Apply colormap
        color = cv2.applyColorMap(normalized, self.colormap)

        # Publish
        out = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
        out.header = msg.header
        pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalColormapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
