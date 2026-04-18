#!/usr/bin/env python3
"""
BESS IMU Timestamp Guard Node

Real-time IMU timestamp guard that prevents SLAM crashes from GQ7 timestamp rewinds.
Based on master-pipeline fix for GLIM IndexedSlidingWindow crashes.

Problem:
- MicroStrain GQ7 occasionally produces out-of-order timestamps
- This causes ~10% of bags to crash in GLIM/FAST-LIO due to:
  * EKF state estimation failures
  * IndexedSlidingWindow assertion errors
  * IMU preintegration NaN propagation

Solution:
- Monitor incoming IMU messages for timestamp rewinds
- Force monotonically increasing timestamps by adding small offset
- Log corrections for debugging

Usage:
  ros2 run slam imu_guard.py

Topics:
  Input:  /imu/data (sensor_msgs/Imu @ 500Hz)
  Output: /imu/data_guarded (sensor_msgs/Imu @ 500Hz, monotonic)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class ImuGuard(Node):
    def __init__(self):
        super().__init__('imu_guard')

        # Parameters
        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_guarded')
        self.declare_parameter('min_increment_ns', 2_000_000)  # 2ms minimum between samples
        self.declare_parameter('warn_interval', 100)  # Log every N corrections
        # Input QoS: 'reliable' for GQ7 (/imu/data), 'best_effort' for
        # Ouster BMI085 (/ouster/imu) — offline bag replay also publishes
        # BEST_EFFORT on /ouster/imu. Wrong reliability = silent drop.
        self.declare_parameter('reliability', 'reliable')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.min_increment_ns = self.get_parameter('min_increment_ns').value
        self.warn_interval = self.get_parameter('warn_interval').value
        self.reliability = self.get_parameter('reliability').value.lower()

        # State
        self.last_raw_ns = 0       # Last raw input timestamp
        self.last_output_ns = 0    # Last output timestamp (always monotonic)
        self.offset_ns = 0         # Correction offset: output = raw + offset
        self.corrections = 0       # Small rewind corrections (GQ7 interleaving)
        self.epoch_changes = 0     # Large time jumps (GPS sync/loss)
        self.total_messages = 0
        self.max_rewind_ns = 0
        self.dropped = 0           # Messages dropped during settling

        # Thresholds
        # Small rewinds (<50ms) = GQ7 dual-source interleaving (raw 104Hz + EKF 500Hz)
        # Large jumps (>1s) = GPS time acquisition/loss, source switch
        self.large_jump_thresh_ns = 1_000_000_000  # 1 second

        # Input QoS must match publisher reliability or messages silently drop.
        # Output stays RELIABLE for FAST-LIO expectations.
        in_rel = (ReliabilityPolicy.BEST_EFFORT if self.reliability == 'best_effort'
                  else ReliabilityPolicy.RELIABLE)
        in_qos = QoSProfile(
            reliability=in_rel,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # Subscriber and publisher
        self.sub = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            in_qos
        )

        self.pub = self.create_publisher(
            Imu,
            self.output_topic,
            imu_qos
        )

        # Diagnostics publisher
        self.diag_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )

        # Periodic diagnostics
        self.create_timer(5.0, self.publish_diagnostics)

        self.get_logger().info(
            f'IMU Guard started: {self.input_topic} -> {self.output_topic}'
        )

    def imu_callback(self, msg: Imu):
        self.total_messages += 1

        # Convert stamp to nanoseconds
        raw_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if self.last_raw_ns == 0:
            # First message — accept as baseline, no offset
            self.last_raw_ns = raw_ns
            self.last_output_ns = raw_ns
            self.offset_ns = 0
            self.pub.publish(msg)
            return

        raw_delta = raw_ns - self.last_raw_ns

        if abs(raw_delta) > self.large_jump_thresh_ns:
            # Large time jump (>1s) — GPS sync, time source switch, or clock reset
            # Recalculate offset so output remains monotonic:
            #   new output = last_output + min_increment
            #   offset = new_output - raw_ns
            self.epoch_changes += 1
            new_output = self.last_output_ns + self.min_increment_ns
            self.offset_ns = new_output - raw_ns
            self.last_raw_ns = raw_ns
            self.last_output_ns = new_output
            direction = "forward" if raw_delta > 0 else "backward"
            self.get_logger().warn(
                f'IMU time epoch change #{self.epoch_changes}: '
                f'{direction} {abs(raw_delta) / 1e9:.2f}s — '
                f'new offset: {self.offset_ns / 1e6:.1f}ms'
            )
            msg.header.stamp.sec = int(new_output // 1_000_000_000)
            msg.header.stamp.nanosec = int(new_output % 1_000_000_000)
            self.pub.publish(msg)
            return

        # Normal-range timestamp. Apply current offset.
        output_ns = raw_ns + self.offset_ns

        if output_ns <= self.last_output_ns:
            # Small rewind after offset — GQ7 interleaving
            # Force monotonic with min increment
            self.corrections += 1
            rewind_ns = self.last_output_ns - output_ns
            self.max_rewind_ns = max(self.max_rewind_ns, rewind_ns)
            output_ns = self.last_output_ns + self.min_increment_ns
            # Update offset to track the correction
            self.offset_ns = output_ns - raw_ns

            if self.corrections % self.warn_interval == 0:
                self.get_logger().warn(
                    f'IMU small rewinds corrected: {self.corrections} '
                    f'(max: {self.max_rewind_ns / 1e6:.2f}ms, '
                    f'offset: {self.offset_ns / 1e6:.1f}ms)'
                )

        self.last_raw_ns = raw_ns
        self.last_output_ns = output_ns
        msg.header.stamp.sec = int(output_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(output_ns % 1_000_000_000)
        self.pub.publish(msg)

    def publish_diagnostics(self):
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'IMU Guard'
        status.hardware_id = 'bess_imu_guard'

        correction_rate = 0.0
        if self.total_messages > 0:
            correction_rate = 100.0 * self.corrections / self.total_messages

        if correction_rate > 1.0:
            status.level = DiagnosticStatus.WARN
            status.message = f'High correction rate: {correction_rate:.2f}%'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Operating normally'

        status.values = [
            KeyValue(key='total_messages', value=str(self.total_messages)),
            KeyValue(key='corrections', value=str(self.corrections)),
            KeyValue(key='correction_rate_pct', value=f'{correction_rate:.3f}'),
            KeyValue(key='max_rewind_ms', value=f'{self.max_rewind_ns / 1e6:.2f}'),
            KeyValue(key='epoch_changes', value=str(self.epoch_changes)),
            KeyValue(key='current_offset_ms', value=f'{self.offset_ns / 1e6:.1f}'),
        ]

        diag.status.append(status)
        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = ImuGuard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
