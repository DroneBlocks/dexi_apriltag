#!/usr/bin/env python3
"""
Test script for visual odometry integration with PX4 EKF2.

Publishes fake VehicleOdometry messages to verify EKF2 is accepting
and fusing the data before testing with real AprilTags.

Modes:
    --ros-args -p mode:=fixed
        Always report drone at origin (0,0,hover_alt).
        Drone should hold position - if it drifts, EKF2 will correct.

    --ros-args -p mode:=track
        Report actual drone position (feedback loop - confirms drift).
        Only useful to verify EKF2 accepts messages.

Usage:
    # Fixed position mode (recommended for testing position hold):
    ros2 run dexi_apriltag test_visual_odometry.py --ros-args -p mode:=fixed

    # Then disable GPS:
    python3 dexi-mavsdk/config/disable_gps.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition
import time
import math


class TestVisualOdometry(Node):
    def __init__(self):
        super().__init__('test_visual_odometry')

        # Parameters
        self.declare_parameter('mode', 'fixed')  # 'fixed' or 'track'
        self.declare_parameter('hover_altitude', 2.0)  # meters (positive = up)
        self.declare_parameter('publish_rate', 50.0)

        # Legacy parameters for compatibility
        self.declare_parameter('tag_north', 0.0)
        self.declare_parameter('tag_east', 0.0)
        self.declare_parameter('tag_down', 0.0)
        self.declare_parameter('use_current_position', True)

        self.mode = self.get_parameter('mode').value
        self.hover_altitude = self.get_parameter('hover_altitude').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Legacy support
        self.tag_north = self.get_parameter('tag_north').value
        self.tag_east = self.get_parameter('tag_east').value
        self.tag_down = self.get_parameter('tag_down').value
        self.use_current_position = self.get_parameter('use_current_position').value

        # QoS for PX4
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Publisher
        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos)

        # Subscriber for current position (to lock origin)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, px4_qos)

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False
        self.origin_locked = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0
        self.publish_count = 0

        # Publish timer
        period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(period, self.publish_odometry)

        self.get_logger().info('Test Visual Odometry started')
        self.get_logger().info(f'Mode: {self.mode.upper()}')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz')

        if self.mode == 'fixed':
            self.get_logger().info(f'FIXED MODE: Always reporting drone at (0, 0, -{self.hover_altitude}) NED')
            self.get_logger().info('Drone should hold position - any drift will be corrected')
        else:
            self.get_logger().info('TRACK MODE: Reporting actual drone position (feedback loop)')
            if self.use_current_position:
                self.get_logger().info('Will lock origin to current position on first message')
            else:
                self.get_logger().info(f'Tag position: [{self.tag_north}, {self.tag_east}, {self.tag_down}] NED')

    def local_position_callback(self, msg):
        """Track current drone position."""
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.position_received = True

    def publish_odometry(self):
        """Publish fake visual odometry."""
        if not self.position_received:
            self.get_logger().info('Waiting for position data...', throttle_duration_sec=1.0)
            return

        if self.mode == 'fixed':
            # FIXED MODE: Always report drone at origin with configured altitude
            # This provides a fixed reference point - if drone drifts, EKF2 will correct
            north = 0.0
            east = 0.0
            down = -self.hover_altitude  # NED: negative = up
        else:
            # TRACK MODE: Report actual position (original behavior)
            # Lock origin on first position
            if self.use_current_position and not self.origin_locked:
                self.origin_x = self.current_x
                self.origin_y = self.current_y
                self.origin_z = self.current_z
                self.origin_locked = True
                self.get_logger().info(
                    f'Origin locked at: [{self.origin_x:.2f}, {self.origin_y:.2f}, {self.origin_z:.2f}] NED')

            if self.use_current_position and self.origin_locked:
                north = self.current_x - self.origin_x
                east = self.current_y - self.origin_y
                down = self.current_z - self.origin_z
            else:
                north = self.current_x - self.tag_north
                east = self.current_y - self.tag_east
                down = self.current_z - self.tag_down

        # Build message
        msg = VehicleOdometry()
        msg.timestamp = int(time.time() * 1e6)
        msg.timestamp_sample = msg.timestamp

        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = [float(north), float(east), float(down)]

        # Identity quaternion
        msg.q = [1.0, 0.0, 0.0, 0.0]

        # Velocity - NaN to let EKF derive
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.angular_velocity = [float('nan'), float('nan'), float('nan')]

        # Variances - trust XY, ignore Z
        msg.position_variance = [0.01, 0.01, 100.0]
        msg.orientation_variance = [0.01, 0.01, 0.01]
        msg.velocity_variance = [float('nan'), float('nan'), float('nan')]

        msg.quality = 100

        self.odom_pub.publish(msg)
        self.publish_count += 1

        # Log every second
        if self.publish_count % int(self.publish_rate) == 0:
            if self.mode == 'fixed':
                self.get_logger().info(
                    f'FIXED: reporting pos=[{north:.2f}, {east:.2f}, {down:.2f}], '
                    f'actual drone=[{self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}]')
            else:
                self.get_logger().info(
                    f'TRACK: pos=[{north:.2f}, {east:.2f}, {down:.2f}], '
                    f'drone=[{self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}]')


def main(args=None):
    rclpy.init(args=args)
    node = TestVisualOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
