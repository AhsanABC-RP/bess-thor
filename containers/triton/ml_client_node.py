#!/usr/bin/env python3
"""
BESS Unified ML Client Node

Single ROS2 node that connects to Triton Inference Server via gRPC and
dispatches inference requests for all ML models, replacing 4 separate
ML containers (pii-mask, segformer, road-damage, thermal-analysis).

Models served:
  - yolov8n_pii:          Face/license plate detection -> PII masking
  - segformer_b2:         Semantic segmentation (19 classes)
  - efficientnet_thermal: Thermal anomaly classification

Subscribes to camera and thermal topics, publishes inference results.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge

try:
    import tritonclient.grpc as grpcclient
    from tritonclient.utils import InferenceServerException
except ImportError:
    raise ImportError("tritonclient[grpc] is required. Install: pip install tritonclient[grpc]")

try:
    import cv2
except ImportError:
    raise ImportError("opencv-python is required. Install: pip install opencv-python-headless")


# Segmentation class labels (Cityscapes 19-class)
SEGMENTATION_CLASSES = [
    "road", "sidewalk", "building", "wall", "fence", "pole",
    "traffic_light", "traffic_sign", "vegetation", "terrain",
    "sky", "person", "rider", "car", "truck", "bus",
    "train", "motorcycle", "bicycle"
]

# Segmentation colormap (Cityscapes palette, BGR)
SEGMENTATION_COLORS = np.array([
    [128, 64, 128], [244, 35, 232], [70, 70, 70], [102, 102, 156],
    [190, 153, 153], [153, 153, 153], [250, 170, 30], [220, 220, 0],
    [107, 142, 35], [152, 251, 152], [70, 130, 180], [220, 20, 60],
    [255, 0, 0], [0, 0, 142], [0, 0, 70], [0, 60, 100],
    [0, 80, 100], [0, 0, 230], [119, 11, 32],
], dtype=np.uint8)

# PII detection class names (YOLO format)
PII_CLASSES = ["face", "license_plate"]

# Thermal anomaly classes
THERMAL_CLASSES = ["normal", "warm_spot", "hot_spot", "leak", "obstruction"]


class BessMLClient(Node):
    """Unified ML inference client connecting to Triton Inference Server."""

    def __init__(self):
        super().__init__("bess_ml_client")

        # -- Parameters --
        self.declare_parameter("triton_url", "localhost:8001")
        self.declare_parameter("enable_pii", True)
        self.declare_parameter("enable_segmentation", True)
        self.declare_parameter("enable_thermal", True)
        self.declare_parameter("pii_confidence_threshold", 0.35)
        self.declare_parameter("thermal_confidence_threshold", 0.5)
        self.declare_parameter("camera_topics", [
            "/camera2/camera_driver/image_raw",
            "/camera3/camera_driver/image_raw",
        ])
        self.declare_parameter("thermal_topics", [
            "/thermal1/camera_driver/image_raw",
            "/thermal2/camera_driver/image_raw",
        ])

        triton_url = self.get_parameter("triton_url").value
        self.enable_pii = self.get_parameter("enable_pii").value
        self.enable_seg = self.get_parameter("enable_segmentation").value
        self.enable_thermal = self.get_parameter("enable_thermal").value
        self.pii_thresh = self.get_parameter("pii_confidence_threshold").value
        self.enable_rdd = False  # road damage removed from stack
        self.thermal_thresh = self.get_parameter("thermal_confidence_threshold").value
        camera_topics = self.get_parameter("camera_topics").value
        thermal_topics = self.get_parameter("thermal_topics").value

        self.bridge = CvBridge()

        # -- Triton client --
        self.get_logger().info(f"Connecting to Triton at {triton_url}")
        try:
            self.triton = grpcclient.InferenceServerClient(url=triton_url)
            if not self.triton.is_server_live():
                self.get_logger().warn("Triton server not yet live, will retry on first request")
        except InferenceServerException as e:
            self.get_logger().error(f"Failed to connect to Triton: {e}")
            self.triton = grpcclient.InferenceServerClient(url=triton_url)

        # -- QoS --
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # -- Subscribers and publishers per camera --
        self.cam_subs = []
        self.pii_pubs = {}        # topic -> publisher (masked image)
        self.pii_comp_pubs = {}   # topic -> publisher (masked compressed)
        self.seg_label_pubs = {}  # topic -> publisher (segmentation labels)
        self.seg_color_pubs = {}  # topic -> publisher (segmentation colored)
        self.rdd_det_pubs = {}    # topic -> publisher (road damage detections)
        self.rdd_ann_pubs = {}    # topic -> publisher (road damage annotated)

        for topic in camera_topics:
            # Derive camera namespace from topic
            # e.g., /camera1/camera_driver/image_raw -> /camera1/camera_driver
            ns = "/".join(topic.rsplit("/", 1)[:-1])

            sub = self.create_subscription(Image, topic, self._make_camera_cb(topic),
                                           sensor_qos)
            self.cam_subs.append(sub)

            if self.enable_pii:
                self.pii_pubs[topic] = self.create_publisher(
                    Image, f"{ns}/image_masked", 5)
                self.pii_comp_pubs[topic] = self.create_publisher(
                    CompressedImage, f"{ns}/image_masked/compressed", 5)

            if self.enable_seg:
                self.seg_label_pubs[topic] = self.create_publisher(
                    Image, f"{ns}/segmentation/labels", 5)
                self.seg_color_pubs[topic] = self.create_publisher(
                    Image, f"{ns}/segmentation/colored", 5)

            if self.enable_rdd:
                cam_name = topic.split("/")[1] if len(topic.split("/")) > 1 else "camera"
                self.rdd_det_pubs[topic] = self.create_publisher(
                    Detection2DArray, f"/road_damage/{cam_name}/detections", 5)
                self.rdd_ann_pubs[topic] = self.create_publisher(
                    Image, f"/road_damage/{cam_name}/annotated", 5)

        # Global road damage detection publisher (merged from all cameras)
        if self.enable_rdd:
            self.rdd_global_pub = self.create_publisher(
                Detection2DArray, "/road_damage/detections", 5)

        # -- Thermal subscribers and publishers --
        self.thermal_subs = []
        self.thermal_heatmap_pubs = {}
        self.thermal_anomaly_pubs = {}

        for topic in thermal_topics:
            ns = "/".join(topic.rsplit("/", 1)[:-1])
            sub = self.create_subscription(Image, topic, self._make_thermal_cb(topic),
                                           sensor_qos)
            self.thermal_subs.append(sub)

            if self.enable_thermal:
                self.thermal_heatmap_pubs[topic] = self.create_publisher(
                    Image, f"/thermal/analysis/heatmap", 5)
                self.thermal_anomaly_pubs[topic] = self.create_publisher(
                    Detection2DArray, f"/thermal/analysis/anomalies", 5)

        models_active = []
        if self.enable_pii:
            models_active.append("yolov8n_pii")
        if self.enable_seg:
            models_active.append("segformer_b2")
        if self.enable_rdd:
            models_active.append("yolov8s_rdd")
        if self.enable_thermal:
            models_active.append("efficientnet_thermal")

        self.get_logger().info(
            f"BESS ML Client initialized. Models: {', '.join(models_active)}. "
            f"Cameras: {len(camera_topics)}, Thermal: {len(thermal_topics)}"
        )

    def _make_camera_cb(self, topic: str):
        """Create a closure callback for a specific camera topic."""
        def callback(msg: Image):
            self._process_camera(topic, msg)
        return callback

    def _make_thermal_cb(self, topic: str):
        """Create a closure callback for a specific thermal topic."""
        def callback(msg: Image):
            self._process_thermal(topic, msg)
        return callback

    # -----------------------------------------------------------------------
    # Camera processing pipeline
    # -----------------------------------------------------------------------

    def _process_camera(self, topic: str, msg: Image):
        """Run PII masking, segmentation, and road damage detection on a camera frame."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV bridge error on {topic}: {e}")
            return

        # PII masking (always runs first so masked image can be published)
        masked_image = cv_image.copy()
        if self.enable_pii:
            masked_image = self._run_pii_masking(topic, cv_image, masked_image, msg.header)

        # Segmentation
        if self.enable_seg:
            self._run_segmentation(topic, cv_image, msg.header)

        # Road damage detection
        if self.enable_rdd:
            self._run_road_damage(topic, cv_image, msg.header)

    def _run_pii_masking(self, topic: str, cv_image: np.ndarray,
                         masked_image: np.ndarray, header: Header) -> np.ndarray:
        """Detect faces/plates and apply Gaussian blur masking."""
        try:
            # Preprocess: resize to 640x640, normalize to FP16
            h_orig, w_orig = cv_image.shape[:2]
            input_img = cv2.resize(cv_image, (640, 640))
            input_img = input_img.astype(np.float32) / 255.0
            input_img = np.transpose(input_img, (2, 0, 1))  # HWC -> CHW
            input_img = np.expand_dims(input_img, axis=0)     # Add batch dim

            # Triton inference
            inputs = [grpcclient.InferInput("images", input_img.shape, "FP32")]
            inputs[0].set_data_from_numpy(input_img)
            outputs = [grpcclient.InferRequestedOutput("output0")]

            result = self.triton.infer(model_name="yolov8n_pii", inputs=inputs,
                                       outputs=outputs)
            detections = result.as_numpy("output0")  # [1, 84, 8400]

            # Parse YOLO detections: transpose to [8400, 84]
            detections = detections[0].T  # [8400, 84]
            # Columns: cx, cy, w, h, class_scores[80+]
            # For PII model: 2 classes (face, license_plate) starting at index 4
            boxes = detections[:, :4]
            scores = detections[:, 4:6]  # 2 PII classes
            max_scores = np.max(scores, axis=1)
            class_ids = np.argmax(scores, axis=1)

            # Filter by confidence
            mask = max_scores > self.pii_thresh
            filtered_boxes = boxes[mask]
            filtered_scores = max_scores[mask]

            # Apply blur to each detection
            scale_x = w_orig / 640.0
            scale_y = h_orig / 640.0

            for box in filtered_boxes:
                cx, cy, w, h = box
                x1 = int((cx - w / 2) * scale_x)
                y1 = int((cy - h / 2) * scale_y)
                x2 = int((cx + w / 2) * scale_x)
                y2 = int((cy + h / 2) * scale_y)

                # Clamp to image bounds
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(w_orig, x2)
                y2 = min(h_orig, y2)

                if x2 > x1 and y2 > y1:
                    roi = masked_image[y1:y2, x1:x2]
                    masked_image[y1:y2, x1:x2] = cv2.GaussianBlur(roi, (99, 99), 30)

            # Publish masked image
            masked_msg = self.bridge.cv2_to_imgmsg(masked_image, encoding="bgr8")
            masked_msg.header = header
            self.pii_pubs[topic].publish(masked_msg)

            # Publish compressed
            comp_msg = CompressedImage()
            comp_msg.header = header
            comp_msg.format = "jpeg"
            _, jpeg_data = cv2.imencode(".jpg", masked_image,
                                         [cv2.IMWRITE_JPEG_QUALITY, 85])
            comp_msg.data = jpeg_data.tobytes()
            self.pii_comp_pubs[topic].publish(comp_msg)

        except InferenceServerException as e:
            self.get_logger().warn(f"PII inference failed on {topic}: {e}", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"PII processing error on {topic}: {e}",
                                    throttle_duration_sec=5)

        return masked_image

    def _run_segmentation(self, topic: str, cv_image: np.ndarray, header: Header):
        """Run SegFormer semantic segmentation."""
        try:
            # Preprocess: resize to 1024x1024, normalize
            input_img = cv2.resize(cv_image, (1024, 1024))
            input_img = input_img.astype(np.float32) / 255.0
            # ImageNet normalization
            mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
            std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
            input_img = (input_img - mean) / std
            input_img = np.transpose(input_img, (2, 0, 1))  # HWC -> CHW
            input_img = np.expand_dims(input_img, axis=0)     # Add batch dim

            inputs = [grpcclient.InferInput("pixel_values", input_img.shape, "FP32")]
            inputs[0].set_data_from_numpy(input_img)
            outputs = [grpcclient.InferRequestedOutput("logits")]

            result = self.triton.infer(model_name="segformer_b2", inputs=inputs,
                                       outputs=outputs)
            logits = result.as_numpy("logits")  # [1, 19, 256, 256]

            # Argmax to get label map
            labels = np.argmax(logits[0], axis=0).astype(np.uint8)  # [256, 256]

            # Resize labels back to original image size
            h_orig, w_orig = cv_image.shape[:2]
            labels_full = cv2.resize(labels, (w_orig, h_orig),
                                     interpolation=cv2.INTER_NEAREST)

            # Publish label image (mono8)
            label_msg = self.bridge.cv2_to_imgmsg(labels_full, encoding="mono8")
            label_msg.header = header
            self.seg_label_pubs[topic].publish(label_msg)

            # Publish colored segmentation
            colored = SEGMENTATION_COLORS[labels_full]  # [H, W, 3]
            colored_msg = self.bridge.cv2_to_imgmsg(colored, encoding="bgr8")
            colored_msg.header = header
            self.seg_color_pubs[topic].publish(colored_msg)

        except InferenceServerException as e:
            self.get_logger().warn(f"Segmentation inference failed on {topic}: {e}",
                                    throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Segmentation error on {topic}: {e}",
                                    throttle_duration_sec=5)

    def _run_road_damage(self, topic: str, cv_image: np.ndarray, header: Header):
        """Run YOLOv8s road damage detection."""
        try:
            h_orig, w_orig = cv_image.shape[:2]
            input_img = cv2.resize(cv_image, (640, 640))
            input_img = input_img.astype(np.float32) / 255.0
            input_img = np.transpose(input_img, (2, 0, 1))
            input_img = np.expand_dims(input_img, axis=0)

            inputs = [grpcclient.InferInput("images", input_img.shape, "FP32")]
            inputs[0].set_data_from_numpy(input_img)
            outputs = [grpcclient.InferRequestedOutput("output0")]

            result = self.triton.infer(model_name="yolov8s_rdd", inputs=inputs,
                                       outputs=outputs)
            detections = result.as_numpy("output0")[0].T  # [8400, 84]

            boxes = detections[:, :4]
            # RDD model: 4 damage classes starting at index 4
            scores = detections[:, 4:8]
            max_scores = np.max(scores, axis=1)
            class_ids = np.argmax(scores, axis=1)

            mask = max_scores > self.rdd_thresh
            filtered_boxes = boxes[mask]
            filtered_scores = max_scores[mask]
            filtered_classes = class_ids[mask]

            scale_x = w_orig / 640.0
            scale_y = h_orig / 640.0

            # Build Detection2DArray message
            det_array = Detection2DArray()
            det_array.header = header
            annotated = cv_image.copy()

            for i, (box, score, cls_id) in enumerate(
                    zip(filtered_boxes, filtered_scores, filtered_classes)):
                cx, cy, w, h = box

                det = Detection2D()
                det.header = header
                det.bbox.center.position.x = float(cx * scale_x)
                det.bbox.center.position.y = float(cy * scale_y)
                det.bbox.size_x = float(w * scale_x)
                det.bbox.size_y = float(h * scale_y)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(cls_id)
                hyp.hypothesis.score = float(score)
                det.results.append(hyp)
                det_array.detections.append(det)

                # Draw on annotated image
                x1 = int((cx - w / 2) * scale_x)
                y1 = int((cy - h / 2) * scale_y)
                x2 = int((cx + w / 2) * scale_x)
                y2 = int((cy + h / 2) * scale_y)
                color = [(0, 0, 255), (0, 165, 255), (0, 255, 255), (255, 0, 0)][cls_id % 4]
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                label = f"{RDD_CLASSES[cls_id]}: {score:.2f}"
                cv2.putText(annotated, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, 1)

            # Publish per-camera detections
            self.rdd_det_pubs[topic].publish(det_array)
            self.rdd_global_pub.publish(det_array)

            # Publish annotated image
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            ann_msg.header = header
            self.rdd_ann_pubs[topic].publish(ann_msg)

        except InferenceServerException as e:
            self.get_logger().warn(f"Road damage inference failed on {topic}: {e}",
                                    throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Road damage error on {topic}: {e}",
                                    throttle_duration_sec=5)

    # -----------------------------------------------------------------------
    # Thermal processing pipeline
    # -----------------------------------------------------------------------

    def _process_thermal(self, topic: str, msg: Image):
        """Run EfficientNet thermal anomaly classification."""
        if not self.enable_thermal:
            return

        try:
            # Thermal images are typically mono16 or mono8
            if msg.encoding in ("mono16", "16UC1"):
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
                # Normalize 16-bit to 0-1 float
                cv_image = cv_image.astype(np.float32) / 65535.0
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                cv_image = cv_image.astype(np.float32) / 255.0

            # Resize to 224x224 for EfficientNet
            input_img = cv2.resize(cv_image, (224, 224))
            input_img = np.expand_dims(input_img, axis=0)  # [1, 224, 224]
            input_img = np.expand_dims(input_img, axis=0)  # [1, 1, 224, 224]

            inputs = [grpcclient.InferInput("input", input_img.shape, "FP32")]
            inputs[0].set_data_from_numpy(input_img)
            outputs_req = [grpcclient.InferRequestedOutput("output")]

            result = self.triton.infer(model_name="efficientnet_thermal", inputs=inputs,
                                       outputs=outputs_req)
            output = result.as_numpy("output")  # [1, 5]
            probs = self._softmax(output[0])

            # Generate heatmap (apply colormap to thermal image)
            h_orig, w_orig = cv_image.shape[:2]
            thermal_norm = (cv_image * 255).astype(np.uint8)
            heatmap = cv2.applyColorMap(thermal_norm, cv2.COLORMAP_INFERNO)

            # Overlay classification text
            class_id = int(np.argmax(probs))
            class_name = THERMAL_CLASSES[class_id]
            confidence = float(probs[class_id])

            if confidence > self.thermal_thresh:
                text = f"{class_name}: {confidence:.2f}"
                cv2.putText(heatmap, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (255, 255, 255), 2)

            # Publish heatmap
            heatmap_msg = self.bridge.cv2_to_imgmsg(heatmap, encoding="bgr8")
            heatmap_msg.header = msg.header
            self.thermal_heatmap_pubs[topic].publish(heatmap_msg)

            # Publish anomaly detections (only if not "normal")
            if class_id > 0 and confidence > self.thermal_thresh:
                det_array = Detection2DArray()
                det_array.header = msg.header
                det = Detection2D()
                det.header = msg.header
                # Whole-image classification
                det.bbox.center.position.x = float(w_orig / 2)
                det.bbox.center.position.y = float(h_orig / 2)
                det.bbox.size_x = float(w_orig)
                det.bbox.size_y = float(h_orig)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(class_id)
                hyp.hypothesis.score = confidence
                det.results.append(hyp)
                det_array.detections.append(det)
                self.thermal_anomaly_pubs[topic].publish(det_array)

        except InferenceServerException as e:
            self.get_logger().warn(f"Thermal inference failed on {topic}: {e}",
                                    throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Thermal error on {topic}: {e}",
                                    throttle_duration_sec=5)

    @staticmethod
    def _softmax(x: np.ndarray) -> np.ndarray:
        """Numerically stable softmax."""
        e_x = np.exp(x - np.max(x))
        return e_x / e_x.sum()


def main(args=None):
    rclpy.init(args=args)
    node = BessMLClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
