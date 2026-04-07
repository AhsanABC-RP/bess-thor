#!/usr/bin/env python3
"""Save FAST-LIO /Odometry to a numpy binary file for offline reinjection."""
import sys, time, signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

class TrajSaver(Node):
    def __init__(self, output):
        super().__init__('traj_saver')
        self.output = output
        self.poses = []  # [(timestamp_ns, x, y, z, qx, qy, qz, qw)]
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Odometry, '/Odometry', self.cb, qos)
        # Save periodically
        self.timer = self.create_timer(10.0, self.save)
        sys.stdout.write('Listening on /Odometry...\n'); sys.stdout.flush()

    def cb(self, msg):
        t = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.poses.append((t, p.x, p.y, p.z, o.x, o.y, o.z, o.w))
        if len(self.poses) % 100 == 0:
            sys.stdout.write(f'{len(self.poses)} poses\n'); sys.stdout.flush()

    def save(self):
        if not self.poses:
            return
        arr = np.array(self.poses, dtype=np.float64)
        np.save(self.output, arr)
        sys.stdout.write(f'SAVED {len(self.poses)} poses to {self.output}\n'); sys.stdout.flush()

def main():
    output = sys.argv[1] if len(sys.argv) > 1 else '/data/maps/trajectory.npy'
    rclpy.init()
    node = TrajSaver(output)
    try:
        rclpy.spin(node)
    except BaseException:
        node.save()

if __name__ == '__main__':
    main()
