#!/usr/bin/env python3
"""Rectify fisheye RGB cameras using calibrated intrinsics.

Subscribes to compressed image topics, undistorts using cv2.fisheye,
publishes rectified images. Uses calibration from DVLC JSON files.

Run inside the extraction container or as standalone node.
"""
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class FisheyeRectifier(Node):
    def __init__(self):
        super().__init__('fisheye_rectifier')
        self.bridge = CvBridge()

        # Load calibrations
        self.cameras = {}
        for cam_name, topic_in, topic_out, calib_path in [
            ('camera2', '/camera2/camera_driver/image_masked/compressed',
             '/camera2/camera_driver/image_rectified/compressed',
             '/opt/bess/config/cameras/calibration/camera2/calibration.json'),
            ('camera3', '/camera3/camera_driver/image_masked/compressed',
             '/camera3/camera_driver/image_rectified/compressed',
             '/opt/bess/config/cameras/calibration/camera3/calibration.json'),
        ]:
            try:
                with open(calib_path) as f:
                    calib = json.load(f)
                K = np.array(calib['K'], dtype=np.float64)
                D = np.array(calib['D'], dtype=np.float64).reshape(4, 1)
                w, h = calib['image_size']

                # Compute undistortion maps once (expensive, do at init)
                # Use balance=0 for no black borders, balance=1 for all pixels
                new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    K, D, (w, h), np.eye(3), balance=0.3)
                map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                    K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)

                self.cameras[cam_name] = {
                    'map1': map1, 'map2': map2,
                    'pub': self.create_publisher(CompressedImage, topic_out, 10),
                }
                self.create_subscription(CompressedImage, topic_in,
                    lambda msg, name=cam_name: self.on_image(msg, name), 10)
                self.get_logger().info(f'{cam_name}: rectification ready ({w}x{h})')
            except Exception as e:
                self.get_logger().error(f'{cam_name}: failed to load calibration: {e}')

    def on_image(self, msg, cam_name):
        cam = self.cameras.get(cam_name)
        if cam is None:
            return

        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        # Rectify
        rectified = cv2.remap(img, cam['map1'], cam['map2'], cv2.INTER_LINEAR)

        # Encode and publish
        _, encoded = cv2.imencode('.jpg', rectified, [cv2.IMWRITE_JPEG_QUALITY, 85])
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = encoded.tobytes()
        cam['pub'].publish(out)


def main():
    rclpy.init()
    node = FisheyeRectifier()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
