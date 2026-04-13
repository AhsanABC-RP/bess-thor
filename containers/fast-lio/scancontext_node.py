#!/usr/bin/env python3
"""
Scan Context Loop Closure Node for FAST-LIO2

Provides place recognition and loop closure detection using Scan Context
descriptors. Works alongside FAST-LIO2 to improve long-term mapping accuracy.

Subscribes:
    /fast_lio/cloud_registered (sensor_msgs/PointCloud2)
    /fast_lio/odometry (nav_msgs/Odometry)

Publishes:
    /fast_lio/loop_closure (geometry_msgs/PoseWithCovarianceStamped)
    /fast_lio/keyframes (visualization_msgs/MarkerArray)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
import struct
from collections import deque
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation


class ScanContextDescriptor:
    """Scan Context descriptor for place recognition."""

    def __init__(self, num_rings=20, num_sectors=60, max_radius=80.0):
        self.num_rings = num_rings
        self.num_sectors = num_sectors
        self.max_radius = max_radius
        self.ring_step = max_radius / num_rings
        self.sector_step = 2 * np.pi / num_sectors

    def compute(self, points: np.ndarray) -> np.ndarray:
        """Compute Scan Context descriptor from point cloud."""
        if len(points) == 0:
            return np.zeros((self.num_rings, self.num_sectors))

        # Convert to polar coordinates
        xy = points[:, :2]
        ranges = np.linalg.norm(xy, axis=1)
        angles = np.arctan2(xy[:, 1], xy[:, 0]) + np.pi  # 0 to 2*pi

        # Filter by range
        valid = ranges < self.max_radius
        ranges = ranges[valid]
        angles = angles[valid]
        heights = points[valid, 2]

        if len(ranges) == 0:
            return np.zeros((self.num_rings, self.num_sectors))

        # Compute ring and sector indices
        ring_idx = np.clip((ranges / self.ring_step).astype(int), 0, self.num_rings - 1)
        sector_idx = np.clip((angles / self.sector_step).astype(int), 0, self.num_sectors - 1)

        # Build descriptor (max height in each bin)
        descriptor = np.zeros((self.num_rings, self.num_sectors))
        for r, s, h in zip(ring_idx, sector_idx, heights):
            descriptor[r, s] = max(descriptor[r, s], h)

        return descriptor

    def distance(self, desc1: np.ndarray, desc2: np.ndarray) -> float:
        """Compute distance between two descriptors with rotation search."""
        min_dist = float('inf')

        for shift in range(self.num_sectors):
            desc2_shifted = np.roll(desc2, shift, axis=1)
            dist = np.linalg.norm(desc1 - desc2_shifted)
            min_dist = min(min_dist, dist)

        return min_dist / (self.num_rings * self.num_sectors)


class Keyframe:
    """Represents a keyframe with pose and descriptor."""

    def __init__(self, idx: int, timestamp: float, position: np.ndarray,
                 orientation: np.ndarray, descriptor: np.ndarray, points: np.ndarray):
        self.idx = idx
        self.timestamp = timestamp
        self.position = position
        self.orientation = orientation
        self.descriptor = descriptor
        self.points = points


class ScanContextNode(Node):
    """ROS2 node for Scan Context loop closure."""

    def __init__(self):
        super().__init__('scancontext_loop_closure')

        # Parameters
        self.declare_parameter('keyframe_dist', 2.0)
        self.declare_parameter('keyframe_angle', 0.2)
        self.declare_parameter('sc_dist_threshold', 0.2)
        self.declare_parameter('min_loop_separation', 100)
        self.declare_parameter('num_candidates', 10)

        self.keyframe_dist = self.get_parameter('keyframe_dist').value
        self.keyframe_angle = self.get_parameter('keyframe_angle').value
        self.sc_dist_threshold = self.get_parameter('sc_dist_threshold').value
        self.min_loop_separation = self.get_parameter('min_loop_separation').value
        self.num_candidates = self.get_parameter('num_candidates').value

        # Scan Context descriptor
        self.sc = ScanContextDescriptor()

        # Keyframe storage
        self.keyframes = []
        self.ring_keys = []  # For fast candidate search
        self.last_keyframe_pose = None
        self.keyframe_idx = 0

        # Current state
        self.current_odom = None
        self.current_cloud = None

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/fast_lio/cloud_registered',
            self.cloud_callback, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/fast_lio/odometry',
            self.odom_callback, sensor_qos
        )

        self.loop_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/fast_lio/loop_closure', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/fast_lio/keyframes', 10
        )

        # Processing timer (10Hz)
        self.create_timer(0.1, self.process_callback)

        self.get_logger().info('Scan Context Loop Closure node started')
        self.get_logger().info(f'  Keyframe dist: {self.keyframe_dist}m')
        self.get_logger().info(f'  SC threshold: {self.sc_dist_threshold}')
        self.get_logger().info(f'  Min loop separation: {self.min_loop_separation}')

    def odom_callback(self, msg: Odometry):
        """Store latest odometry."""
        self.current_odom = msg

    def cloud_callback(self, msg: PointCloud2):
        """Store latest point cloud."""
        self.current_cloud = msg

    def parse_pointcloud(self, msg: PointCloud2) -> np.ndarray:
        """Parse PointCloud2 to numpy array."""
        # Find XYZ field offsets
        x_offset = y_offset = z_offset = None
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        if None in (x_offset, y_offset, z_offset):
            return np.array([])

        # Parse points
        points = []
        data = msg.data
        point_step = msg.point_step

        for i in range(msg.width * msg.height):
            offset = i * point_step
            x = struct.unpack_from('f', data, offset + x_offset)[0]
            y = struct.unpack_from('f', data, offset + y_offset)[0]
            z = struct.unpack_from('f', data, offset + z_offset)[0]
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append([x, y, z])

        return np.array(points) if points else np.array([])

    def should_add_keyframe(self, position: np.ndarray, orientation: np.ndarray) -> bool:
        """Check if we should add a new keyframe."""
        if self.last_keyframe_pose is None:
            return True

        last_pos, last_ori = self.last_keyframe_pose

        # Check distance
        dist = np.linalg.norm(position - last_pos)
        if dist > self.keyframe_dist:
            return True

        # Check angle
        r1 = Rotation.from_quat(last_ori)
        r2 = Rotation.from_quat(orientation)
        angle = (r1.inv() * r2).magnitude()
        if angle > self.keyframe_angle:
            return True

        return False

    def find_loop_candidates(self, descriptor: np.ndarray, current_idx: int) -> list:
        """Find loop closure candidates using ring key."""
        if len(self.keyframes) < self.min_loop_separation:
            return []

        # Ring key: mean of each ring (for fast filtering)
        ring_key = descriptor.mean(axis=1)

        # Find candidates using ring key similarity
        candidates = []
        for i, kf in enumerate(self.keyframes):
            # Skip recent keyframes
            if current_idx - kf.idx < self.min_loop_separation:
                continue

            # Ring key distance (fast filter)
            ring_dist = np.linalg.norm(ring_key - self.ring_keys[i])
            if ring_dist < 0.5:  # Threshold for ring key
                candidates.append((i, ring_dist))

        # Sort by ring key distance and take top candidates
        candidates.sort(key=lambda x: x[1])
        return [c[0] for c in candidates[:self.num_candidates]]

    def detect_loop(self, descriptor: np.ndarray, candidates: list) -> tuple:
        """Detect loop closure among candidates."""
        best_match = None
        best_dist = float('inf')

        for idx in candidates:
            kf = self.keyframes[idx]
            dist = self.sc.distance(descriptor, kf.descriptor)

            if dist < best_dist and dist < self.sc_dist_threshold:
                best_dist = dist
                best_match = idx

        return best_match, best_dist

    def process_callback(self):
        """Process current data and detect loops."""
        if self.current_odom is None or self.current_cloud is None:
            return

        # Extract pose
        pos = self.current_odom.pose.pose.position
        ori = self.current_odom.pose.pose.orientation
        position = np.array([pos.x, pos.y, pos.z])
        orientation = np.array([ori.x, ori.y, ori.z, ori.w])

        # Check if we should add keyframe
        if not self.should_add_keyframe(position, orientation):
            return

        # Parse point cloud
        points = self.parse_pointcloud(self.current_cloud)
        if len(points) < 100:
            return

        # Compute descriptor
        descriptor = self.sc.compute(points)

        # Find and check loop candidates
        candidates = self.find_loop_candidates(descriptor, self.keyframe_idx)

        if candidates:
            match_idx, match_dist = self.detect_loop(descriptor, candidates)

            if match_idx is not None:
                matched_kf = self.keyframes[match_idx]
                self.get_logger().info(
                    f'Loop detected! KF {self.keyframe_idx} -> KF {matched_kf.idx} '
                    f'(dist: {match_dist:.3f})'
                )

                # Publish loop closure
                self.publish_loop_closure(matched_kf, position, orientation)

        # Add keyframe
        kf = Keyframe(
            self.keyframe_idx,
            self.current_odom.header.stamp.sec + self.current_odom.header.stamp.nanosec * 1e-9,
            position, orientation, descriptor, points
        )
        self.keyframes.append(kf)
        self.ring_keys.append(descriptor.mean(axis=1))
        self.last_keyframe_pose = (position, orientation)
        self.keyframe_idx += 1

        # Publish keyframe markers
        self.publish_keyframe_markers()

    def publish_loop_closure(self, matched_kf: Keyframe,
                            current_pos: np.ndarray, current_ori: np.ndarray):
        """Publish loop closure constraint."""
        msg = PoseWithCovarianceStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        # Relative pose from current to matched keyframe
        msg.pose.pose.position.x = matched_kf.position[0]
        msg.pose.pose.position.y = matched_kf.position[1]
        msg.pose.pose.position.z = matched_kf.position[2]
        msg.pose.pose.orientation.x = matched_kf.orientation[0]
        msg.pose.pose.orientation.y = matched_kf.orientation[1]
        msg.pose.pose.orientation.z = matched_kf.orientation[2]
        msg.pose.pose.orientation.w = matched_kf.orientation[3]

        # Covariance (diagonal)
        cov = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        msg.pose.covariance = cov

        self.loop_pub.publish(msg)

    def publish_keyframe_markers(self):
        """Publish visualization markers for keyframes."""
        marker_array = MarkerArray()

        # Keyframe positions
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'keyframes'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        for kf in self.keyframes[-100:]:  # Last 100 keyframes
            from geometry_msgs.msg import Point
            p = Point()
            p.x = kf.position[0]
            p.y = kf.position[1]
            p.z = kf.position[2]
            marker.points.append(p)

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main():
    rclpy.init()
    node = ScanContextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
