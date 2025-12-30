#!/usr/bin/env python3
"""
AprilTag Visual Odometry for PX4 EKF2 fusion.

Publishes AprilTag pose as VehicleOdometry messages to enable
drift-free position hold when the tag is visible.

Usage:
    ros2 run dexi_apriltag apriltag_odometry.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleAttitude
import time
import math
from collections import deque


class AprilTagOdometry(Node):
    def __init__(self):
        super().__init__('apriltag_odometry')

        # Parameters
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('publish_rate', 10.0)  # Hz - slower to reduce oscillation
        self.declare_parameter('position_variance', [2.0, 2.0, 100.0])  # Higher variance - less aggressive corrections
        self.declare_parameter('orientation_variance', [0.01, 0.01, 0.01])
        self.declare_parameter('filter_length', 5)  # Moving average filter length (ModalAI default)
        self.declare_parameter('dry_run', False)  # If true, log but don't publish to EKF2

        self.tag_family = self.get_parameter('tag_family').value
        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.position_variance = self.get_parameter('position_variance').value
        self.orientation_variance = self.get_parameter('orientation_variance').value
        self.filter_length = self.get_parameter('filter_length').value
        self.dry_run = self.get_parameter('dry_run').value

        # QoS for PX4
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # TF2 for AprilTag position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for visual odometry to EKF2
        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos)

        # Subscriber for drone heading and position
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, px4_qos)

        # Subscriber for drone attitude (pitch/roll for tilt compensation)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_callback, px4_qos)

        # State
        self.drone_heading = 0.0
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_pitch = 0.0
        self.drone_roll = 0.0
        self.heading_valid = False
        self.position_valid = False
        self.attitude_valid = False
        self.tag_visible = False
        self.last_publish_time = None

        # Origin offset - aligns tag frame with EKF2 frame
        self.origin_locked = False
        self.offset_north = 0.0
        self.offset_east = 0.0
        self.offset_down = 0.0
        # Track EKF2 position to detect frame jumps (e.g., GPS disable)
        self.last_ekf2_x = None
        self.last_ekf2_y = None
        self.last_ekf2_time = None

        # Moving average filter buffers (ModalAI approach)
        # Filter smooths position estimates to prevent EKF2 discontinuities
        self.north_buffer = deque(maxlen=self.filter_length)
        self.east_buffer = deque(maxlen=self.filter_length)
        self.down_buffer = deque(maxlen=self.filter_length)

        # Publish timer
        period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(period, self.publish_odometry)

        self.get_logger().info(f'AprilTag Odometry initialized')
        self.get_logger().info(f'Tracking tag: {self.tag_family}:{self.target_tag_id}')
        self.get_logger().info(f'Filter: {self.filter_length}-sample moving average')
        if self.dry_run:
            self.get_logger().warn(f'DRY RUN MODE - logging only, NOT publishing to EKF2')
        else:
            self.get_logger().info(f'Publishing at {self.publish_rate} Hz to /fmu/in/vehicle_visual_odometry')

    def local_position_callback(self, msg):
        """Track drone heading and position for frame alignment."""
        self.drone_heading = msg.heading
        self.drone_x = msg.x  # North
        self.drone_y = msg.y  # East
        self.drone_z = msg.z  # Down
        self.heading_valid = True
        self.position_valid = True

    def attitude_callback(self, msg):
        """Track drone attitude for tilt compensation with body-fixed camera."""
        # PX4 quaternion is [w, x, y, z]
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]

        # Extract roll (rotation around X/forward axis)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        self.drone_roll = math.atan2(sinr_cosp, cosr_cosp)

        # Extract pitch (rotation around Y/right axis)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.drone_pitch = math.copysign(math.pi / 2, sinp)
        else:
            self.drone_pitch = math.asin(sinp)

        self.attitude_valid = True

    def get_tag_transform(self):
        """
        Get AprilTag position from TF.

        Returns (success, x, y, z, qw, qx, qy, qz) where x,y,z are in body frame.
        """
        tag_frame = f'{self.tag_family}:{self.target_tag_id}'
        try:
            # Use Time(0) to get latest available transform regardless of timestamp
            # This avoids TF_OLD_DATA issues with sim time vs wall clock
            transform = self.tf_buffer.lookup_transform(
                'base_link', tag_frame, rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.05))

            # Store TF timestamp for use in odometry message
            self._tf_timestamp_us = int(transform.header.stamp.sec * 1e6 +
                                        transform.header.stamp.nanosec / 1e3)

            # Track last valid transform time to detect stale data
            tf_stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            if not hasattr(self, '_last_tf_stamp'):
                self._last_tf_stamp = tf_stamp
            elif tf_stamp == self._last_tf_stamp:
                # Same timestamp as before - TF hasn't updated
                if not hasattr(self, '_stale_count'):
                    self._stale_count = 0
                self._stale_count += 1
                if self._stale_count > 5:  # 5 consecutive stale reads = ~100ms at 50Hz
                    self.get_logger().debug('TF stale - no new data', throttle_duration_sec=1.0)
                    return False, 0, 0, 0, 1, 0, 0, 0
            else:
                self._last_tf_stamp = tf_stamp
                self._stale_count = 0

            # Extract translation (tag position relative to drone)
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # Extract rotation
            qw = transform.transform.rotation.w
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z

            return True, tx, ty, tz, qw, qx, qy, qz

        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}', throttle_duration_sec=1.0)
            return False, 0, 0, 0, 1, 0, 0, 0

    def publish_odometry(self):
        """Publish visual odometry if tag is visible."""
        if not self.heading_valid or not self.position_valid or not self.attitude_valid:
            self.get_logger().debug('Waiting for heading/position/attitude...', throttle_duration_sec=1.0)
            return

        success, tx, ty, tz, qw, qx, qy, qz = self.get_tag_transform()

        if not success:
            if self.tag_visible:
                self.get_logger().info('Tag lost - stopping odometry publishing')
                self.tag_visible = False
            return

        if not self.tag_visible:
            self.get_logger().info('Tag detected - starting odometry publishing')
            self.tag_visible = True

        # Transform from TF frame to body frame with TILT COMPENSATION
        # TF gives tag position in drone's body frame. For odometry, we need
        # drone position relative to tag origin, which is the opposite.
        # From precision_landing.py empirical mapping:
        #   TF X = altitude (down) → drone above tag = -tx
        #   TF Y = south → drone north of tag = -ty
        #   TF Z = west → drone east of tag = -tz
        #
        # TILT COMPENSATION disabled for testing
        # TODO: Re-enable once basic tracking is stable
        # altitude = tx
        # tilt_forward_shift = altitude * math.tan(self.drone_pitch)
        # tilt_right_shift = altitude * math.tan(self.drone_roll)
        # body_forward = -ty + tilt_forward_shift
        # body_right = -tz + tilt_right_shift

        body_forward = -ty
        body_right = -tz
        body_down = -tx

        # Detect EKF2 frame jumps (e.g., when GPS is disabled mid-flight)
        # Look for sudden jumps in EKF2's reported position, NOT in our calculations
        # This distinguishes GPS-disable events from normal yaw/movement
        now = time.time()
        if self.origin_locked and self.last_ekf2_x is not None:
            dt = now - self.last_ekf2_time
            if dt > 0 and dt < 1.0:  # Only check if we have recent data
                dx = abs(self.drone_x - self.last_ekf2_x)
                dy = abs(self.drone_y - self.last_ekf2_y)
                # 5 m/s is very fast - if EKF2 position changes faster than this
                # in a single update, it's likely a frame jump not real movement
                velocity = math.sqrt(dx*dx + dy*dy) / dt
                if velocity > 5.0 and (dx > 0.5 or dy > 0.5):
                    self.get_logger().warn(
                        f'EKF2 frame jump detected (moved {dx:.2f}, {dy:.2f} in {dt:.3f}s) - re-locking origin')
                    self.origin_locked = False
        self.last_ekf2_x = self.drone_x
        self.last_ekf2_y = self.drone_y
        self.last_ekf2_time = now

        # Calculate body-to-NED transform using current heading
        cos_h = math.cos(self.drone_heading)
        sin_h = math.sin(self.drone_heading)
        tag_rel_north = body_forward * cos_h - body_right * sin_h
        tag_rel_east = body_forward * sin_h + body_right * cos_h

        # Lock origin offset on first tag detection or after frame jump
        if not self.origin_locked:
            self.offset_north = self.drone_x - tag_rel_north
            self.offset_east = self.drone_y - tag_rel_east
            self.offset_down = self.drone_z - body_down
            self.origin_locked = True
            # Clear filter buffers to prevent old data from affecting new origin
            self.north_buffer.clear()
            self.east_buffer.clear()
            self.down_buffer.clear()
            self.get_logger().info(
                f'Origin locked - heading: {math.degrees(self.drone_heading):.1f}°, '
                f'offset: [{self.offset_north:.2f}, {self.offset_east:.2f}, {self.offset_down:.2f}]')

        # Apply offset to get position in EKF2's frame
        north_raw = tag_rel_north + self.offset_north
        east_raw = tag_rel_east + self.offset_east
        down_raw = body_down + self.offset_down

        # Apply moving average filter (ModalAI approach)
        # This smooths position estimates to prevent EKF2 discontinuities
        self.north_buffer.append(north_raw)
        self.east_buffer.append(east_raw)
        self.down_buffer.append(down_raw)

        north = sum(self.north_buffer) / len(self.north_buffer)
        east = sum(self.east_buffer) / len(self.east_buffer)
        down = sum(self.down_buffer) / len(self.down_buffer)

        # Build VehicleOdometry message
        msg = VehicleOdometry()
        # Use TF timestamp (when measurement was taken), not wall clock
        # This ensures EKF2 applies the correction at the right time
        msg.timestamp = self._tf_timestamp_us
        msg.timestamp_sample = self._tf_timestamp_us

        # Position in NED frame (tag is at origin)
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = [float(north), float(east), float(down)]

        # Orientation - set to NaN to indicate we're not providing yaw
        # AprilTag yaw is ambiguous when viewed from above
        msg.q = [float('nan'), float('nan'), float('nan'), float('nan')]

        # Velocity - set to NaN to let EKF2 derive from position
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        msg.velocity = [float('nan'), float('nan'), float('nan')]

        # Angular velocity - not provided
        msg.angular_velocity = [float('nan'), float('nan'), float('nan')]

        # Variances
        msg.position_variance = [
            float(self.position_variance[0]),
            float(self.position_variance[1]),
            float(self.position_variance[2])
        ]
        msg.orientation_variance = [
            float(self.orientation_variance[0]),
            float(self.orientation_variance[1]),
            float(self.orientation_variance[2])
        ]
        msg.velocity_variance = [float('nan'), float('nan'), float('nan')]

        # Quality indicator (0-100)
        msg.quality = 100

        # Only publish if not in dry run mode
        if not self.dry_run:
            self.odom_pub.publish(msg)

        # Log periodically for debugging
        now = time.time()
        if self.last_publish_time is None or now - self.last_publish_time > 1.0:
            heading_deg = math.degrees(self.drone_heading)
            if heading_deg < 0:
                heading_deg += 360  # Normalize to 0-360° to match HUD
            self.get_logger().info(
                f'Body: [{body_forward:.2f}, {body_right:.2f}] | '
                f'NED: [{north:.2f}, {east:.2f}] | '
                f'Hdg: {heading_deg:.0f}°')
            self.last_publish_time = now


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagOdometry()

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
