#!/usr/bin/env python3
"""
BESS Tesla CAN Decoder Node
Decodes Tesla vehicle CAN messages to ROS2 odometry

Tesla CAN IDs (Model 3/Y):
- 0x118: DI_speed (vehicle speed)
- 0x129: DI_torque (steering angle)
- 0x2B6: UI_vehicleControl (drive state)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from can_msgs.msg import Frame
import math
import struct

class TeslaCanDecoder(Node):
    def __init__(self):
        super().__init__('tesla_can_decoder')

        # Parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('wheelbase', 2.875)  # Tesla Model 3 wheelbase in meters

        self.wheelbase = self.get_parameter('wheelbase').value

        # State
        self.vehicle_speed = 0.0  # m/s
        self.steering_angle = 0.0  # radians
        self.last_speed_time = None

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.can_sub = self.create_subscription(
            Frame,
            '/can/rx',
            self.can_callback,
            qos
        )

        # Publishers
        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/vehicle/twist',
            10
        )
        self.speed_pub = self.create_publisher(Float64, '/vehicle/speed', 10)
        self.steering_pub = self.create_publisher(Float64, '/vehicle/steering_angle', 10)

        # Timer for publishing odometry
        self.timer = self.create_timer(0.02, self.publish_twist)  # 50Hz

        self.get_logger().info('Tesla CAN Decoder initialized')

    def can_callback(self, msg: Frame):
        """Process incoming CAN frames"""
        can_id = msg.id
        data = bytes(msg.data)

        if can_id == 0x118:
            # DI_speed - Vehicle speed
            # Bytes 2-3: speed in 0.01 km/h
            speed_raw = struct.unpack('>H', data[2:4])[0]
            self.vehicle_speed = (speed_raw * 0.01) / 3.6  # Convert to m/s
            self.last_speed_time = self.get_clock().now()

            # Publish speed
            speed_msg = Float64()
            speed_msg.data = self.vehicle_speed
            self.speed_pub.publish(speed_msg)

        elif can_id == 0x129:
            # DI_torque - Contains steering angle
            # Bytes 0-1: steering angle in 0.1 degrees
            angle_raw = struct.unpack('>h', data[0:2])[0]  # Signed
            self.steering_angle = math.radians(angle_raw * 0.1)

            # Publish steering angle
            angle_msg = Float64()
            angle_msg.data = self.steering_angle
            self.steering_pub.publish(angle_msg)

    def publish_twist(self):
        """Publish vehicle twist with covariance"""
        now = self.get_clock().now()

        # Create twist message
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = now.to_msg()
        twist_msg.header.frame_id = 'base_link'

        # Linear velocity (forward)
        twist_msg.twist.twist.linear.x = self.vehicle_speed
        twist_msg.twist.twist.linear.y = 0.0
        twist_msg.twist.twist.linear.z = 0.0

        # Angular velocity (yaw rate from bicycle model)
        if abs(self.steering_angle) > 0.001:
            yaw_rate = self.vehicle_speed * math.tan(self.steering_angle) / self.wheelbase
        else:
            yaw_rate = 0.0

        twist_msg.twist.twist.angular.x = 0.0
        twist_msg.twist.twist.angular.y = 0.0
        twist_msg.twist.twist.angular.z = yaw_rate

        # Covariance (diagonal)
        cov = [0.0] * 36
        cov[0] = 0.01  # x velocity variance
        cov[7] = 0.01  # y velocity variance
        cov[14] = 0.01  # z velocity variance
        cov[21] = 0.01  # roll rate variance
        cov[28] = 0.01  # pitch rate variance
        cov[35] = 0.05  # yaw rate variance
        twist_msg.twist.covariance = cov

        self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeslaCanDecoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
