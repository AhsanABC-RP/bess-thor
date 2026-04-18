#!/usr/bin/env python3
"""
Ouster IMU Timestamp Guard — forces monotonic timestamps on /ouster/imu.

The Ouster BMI085 internal IMU in TIME_FROM_ROS_TIME mode stamps packets
by host reception time. With imu_packets_per_frame=8, adjacent samples can
differ <1ms and occasionally rewind by up to ~11ms. FAST-LIO2's patched
tolerance is 10ms, so rewinds >10ms clear the IMU buffer and crash the EKF.

This guard is modelled on the GQ7 imu_guard.py from bess_platform but uses
SensorDataQoS (BEST_EFFORT) on the subscriber to match Ouster's QoS.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class OusterImuGuard(Node):
    def __init__(self):
        super().__init__('ouster_imu_guard')

        self.declare_parameter('input_topic', '/ouster/imu')
        self.declare_parameter('output_topic', '/ouster/imu_guarded')
        self.declare_parameter('min_increment_ns', 500_000)  # 0.5ms — Ouster BMI085 runs ~640Hz

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.min_increment_ns = self.get_parameter('min_increment_ns').value

        self.last_raw_ns = 0
        self.last_output_ns = 0
        self.offset_ns = 0
        self.corrections = 0
        self.total = 0

        self.sub = self.create_subscription(
            Imu,
            self.input_topic,
            self.callback,
            qos_profile_sensor_data,
        )

        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.pub = self.create_publisher(Imu, self.output_topic, out_qos)

        self.get_logger().info(
            f'Ouster IMU Guard: {self.input_topic} -> {self.output_topic}'
        )

    def callback(self, msg: Imu):
        self.total += 1
        raw_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if self.last_raw_ns == 0:
            self.last_raw_ns = raw_ns
            self.last_output_ns = raw_ns
            self.pub.publish(msg)
            return

        raw_delta = raw_ns - self.last_raw_ns

        if abs(raw_delta) > 5_000_000_000:
            self.offset_ns = (self.last_output_ns + self.min_increment_ns) - raw_ns
            self.last_raw_ns = raw_ns
            self.last_output_ns = self.last_output_ns + self.min_increment_ns
            self.get_logger().warn(f'Large jump {raw_delta/1e9:.2f}s, new offset {self.offset_ns/1e6:.1f}ms')
            out_ns = self.last_output_ns
        else:
            out_ns = raw_ns + self.offset_ns
            if out_ns <= self.last_output_ns:
                self.corrections += 1
                out_ns = self.last_output_ns + self.min_increment_ns
                self.offset_ns = out_ns - raw_ns
                if self.corrections % 500 == 0:
                    pct = 100.0 * self.corrections / self.total
                    self.get_logger().warn(
                        f'Rewinds corrected: {self.corrections} ({pct:.1f}%)'
                    )
            self.last_raw_ns = raw_ns
            self.last_output_ns = out_ns

        msg.header.stamp.sec = int(out_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(out_ns % 1_000_000_000)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OusterImuGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
