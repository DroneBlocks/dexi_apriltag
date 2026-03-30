#!/usr/bin/env python3
"""
AprilTag Visual Odometry for PX4 EKF2 fusion.

Publishes AprilTag pose as VehicleOdometry messages to enable
drift-free position hold when tags are visible.

Supports two modes:
  - Single tag mode (legacy): Tracks one tag, uses origin offset alignment
  - Tag map mode: Multiple tags at known world positions, provides absolute position

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
        self.declare_parameter('position_variance', [10.0, 10.0, 100.0])  # High variance - gentle EKF2 corrections
        self.declare_parameter('orientation_variance', [0.01, 0.01, 0.01])
        self.declare_parameter('filter_length', 5)  # Moving average filter length (ModalAI default)
        self.declare_parameter('dry_run', False)  # If true, log but don't publish to EKF2

        # Tag map parameters (empty = single tag mode for backward compatibility)
        self.declare_parameter('tag_map_ids', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('tag_map_x', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('tag_map_y', rclpy.Parameter.Type.DOUBLE_ARRAY)

        self.tag_family = self.get_parameter('tag_family').value
        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.position_variance = self.get_parameter('position_variance').value
        self.orientation_variance = self.get_parameter('orientation_variance').value
        self.filter_length = self.get_parameter('filter_length').value
        self.dry_run = self.get_parameter('dry_run').value

        # Build tag map
        self.tag_map = {}
        self.use_tag_map = False
        try:
            tag_map_ids = self.get_parameter('tag_map_ids').value
            tag_map_x = self.get_parameter('tag_map_x').value
            tag_map_y = self.get_parameter('tag_map_y').value
            if tag_map_ids and tag_map_x and tag_map_y:
                if len(tag_map_ids) == len(tag_map_x) == len(tag_map_y):
                    self.tag_map = {
                        int(tid): (float(x), float(y))
                        for tid, x, y in zip(tag_map_ids, tag_map_x, tag_map_y)
                    }
                    self.use_tag_map = True
                else:
                    self.get_logger().error('Tag map arrays must be the same length')
        except rclpy.exceptions.ParameterUninitializedException:
            pass  # No tag map provided, single tag mode

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
        self.last_visible_tag_id = None
        self.current_tag_id = None  # Tag we're actively tracking
        self.current_tag_distance = float('inf')

        # Origin offset - aligns tag/map frame with EKF2 frame
        self.origin_locked = False
        self.offset_north = 0.0
        self.offset_east = 0.0
        self.offset_down = 0.0

        # Moving average filter buffers (ModalAI approach)
        # Filter smooths position estimates to prevent EKF2 discontinuities
        self.north_buffer = deque(maxlen=self.filter_length)
        self.east_buffer = deque(maxlen=self.filter_length)
        self.down_buffer = deque(maxlen=self.filter_length)

        # Publish timer
        period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(period, self.publish_odometry)

        self.get_logger().info(f'AprilTag Odometry initialized')
        if self.use_tag_map:
            self.get_logger().info(f'Tag map mode: {len(self.tag_map)} tags')
            for tid, (x, y) in sorted(self.tag_map.items()):
                self.get_logger().info(f'  Tag {tid}: ({x:.2f}, {y:.2f})')
        else:
            self.get_logger().info(f'Single tag mode: {self.tag_family}:{self.target_tag_id}')
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

    def _lookup_tag_tf(self, tag_id):
        """Look up TF for a specific tag ID. Returns (transform, distance) or None."""
        tag_frame = f'{self.tag_family}:{tag_id}'
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', tag_frame, rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.01))

            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            distance = math.sqrt(tx * tx + ty * ty + tz * tz)
            return transform, distance
        except Exception:
            return None

    def get_tag_transform(self):
        """
        Get the best visible AprilTag position from TF.

        In tag map mode: checks all tags in the map, picks the closest.
        In single tag mode: looks up the single target tag.

        Returns (success, tag_id, tx, ty, tz, qw, qx, qy, qz)
        """
        if self.use_tag_map:
            # Collect all visible tags
            visible_tags = {}
            for tag_id in self.tag_map:
                result = self._lookup_tag_tf(tag_id)
                if result is not None:
                    visible_tags[tag_id] = result

            if not visible_tags:
                self.current_tag_id = None
                self.current_tag_distance = float('inf')
                return False, -1, 0, 0, 0, 1, 0, 0, 0

            # Tag switching with hysteresis: stick with current tag unless
            # another tag is >30% closer (prevents rapid bouncing)
            best_tag_id = min(visible_tags, key=lambda tid: visible_tags[tid][1])
            best_distance = visible_tags[best_tag_id][1]

            if (self.current_tag_id is not None and
                    self.current_tag_id in visible_tags):
                current_distance = visible_tags[self.current_tag_id][1]
                # Only switch if new tag is significantly closer
                if best_distance < current_distance * 0.7:
                    chosen_id = best_tag_id
                else:
                    chosen_id = self.current_tag_id
            else:
                chosen_id = best_tag_id

            self.current_tag_id = chosen_id
            self.current_tag_distance = visible_tags[chosen_id][1]
            best_transform = visible_tags[chosen_id][0]

            t = best_transform.transform
            # Store TF timestamp
            self._tf_timestamp_us = int(
                best_transform.header.stamp.sec * 1e6 +
                best_transform.header.stamp.nanosec / 1e3)

            # Stale detection
            tf_stamp = (best_transform.header.stamp.sec +
                        best_transform.header.stamp.nanosec / 1e9)
            if not hasattr(self, '_last_tf_stamp'):
                self._last_tf_stamp = tf_stamp
            elif tf_stamp == self._last_tf_stamp:
                if not hasattr(self, '_stale_count'):
                    self._stale_count = 0
                self._stale_count += 1
                if self._stale_count > 5:
                    self.get_logger().debug('TF stale - no new data', throttle_duration_sec=1.0)
                    return False, -1, 0, 0, 0, 1, 0, 0, 0
            else:
                self._last_tf_stamp = tf_stamp
                self._stale_count = 0

            return (True, chosen_id,
                    t.translation.x, t.translation.y, t.translation.z,
                    t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z)

        else:
            # Single tag mode (legacy)
            result = self._lookup_tag_tf(self.target_tag_id)
            if result is None:
                return False, -1, 0, 0, 0, 1, 0, 0, 0

            transform, _ = result

            # Store TF timestamp
            self._tf_timestamp_us = int(
                transform.header.stamp.sec * 1e6 +
                transform.header.stamp.nanosec / 1e3)

            # Stale detection
            tf_stamp = (transform.header.stamp.sec +
                        transform.header.stamp.nanosec / 1e9)
            if not hasattr(self, '_last_tf_stamp'):
                self._last_tf_stamp = tf_stamp
            elif tf_stamp == self._last_tf_stamp:
                if not hasattr(self, '_stale_count'):
                    self._stale_count = 0
                self._stale_count += 1
                if self._stale_count > 5:
                    self.get_logger().debug('TF stale - no new data', throttle_duration_sec=1.0)
                    return False, -1, 0, 0, 0, 1, 0, 0, 0
            else:
                self._last_tf_stamp = tf_stamp
                self._stale_count = 0

            t = transform.transform
            return (True, self.target_tag_id,
                    t.translation.x, t.translation.y, t.translation.z,
                    t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z)

    def publish_odometry(self):
        """Publish visual odometry if tag is visible."""
        if not self.heading_valid or not self.position_valid or not self.attitude_valid:
            self.get_logger().debug('Waiting for heading/position/attitude...', throttle_duration_sec=1.0)
            return

        success, tag_id, tx, ty, tz, qw, qx, qy, qz = self.get_tag_transform()

        if not success:
            if self.tag_visible:
                self.get_logger().info('Tag lost - stopping odometry publishing')
                self.tag_visible = False
                self.last_visible_tag_id = None
            return

        if not self.tag_visible:
            self.get_logger().info(f'Tag {tag_id} detected - starting odometry publishing')
            self.tag_visible = True

        if tag_id != self.last_visible_tag_id:
            if self.last_visible_tag_id is not None:
                self.get_logger().info(f'Switched to tag {tag_id}')
            self.last_visible_tag_id = tag_id

        # Transform from TF frame to body frame
        # TF X = altitude (down), TF Y = south, TF Z = west
        body_forward = -ty
        body_right = -tz
        body_down = -tx

        # Calculate body-to-NED transform using current heading
        cos_h = math.cos(self.drone_heading)
        sin_h = math.sin(self.drone_heading)
        tag_rel_north = body_forward * cos_h - body_right * sin_h
        tag_rel_east = body_forward * sin_h + body_right * cos_h

        if self.use_tag_map:
            # Tag map mode: compute absolute position in map frame
            tag_world_north, tag_world_east = self.tag_map[tag_id]
            drone_map_north = tag_world_north + tag_rel_north
            drone_map_east = tag_world_east + tag_rel_east

            # Lock offset to align map frame with EKF2 frame on first detection
            if not self.origin_locked:
                self.offset_north = self.drone_x - drone_map_north
                self.offset_east = self.drone_y - drone_map_east
                self.offset_down = self.drone_z - body_down
                self.origin_locked = True
                self.north_buffer.clear()
                self.east_buffer.clear()
                self.down_buffer.clear()
                self.get_logger().info(
                    f'Origin locked on tag {tag_id} - heading: {math.degrees(self.drone_heading):.1f}°, '
                    f'offset: [{self.offset_north:.2f}, {self.offset_east:.2f}, {self.offset_down:.2f}]')

            north_raw = drone_map_north + self.offset_north
            east_raw = drone_map_east + self.offset_east
            down_raw = body_down + self.offset_down

        else:
            # Single tag mode (legacy): tag is implicitly at origin
            if not self.origin_locked:
                self.offset_north = self.drone_x - tag_rel_north
                self.offset_east = self.drone_y - tag_rel_east
                self.offset_down = self.drone_z - body_down
                self.origin_locked = True
                self.north_buffer.clear()
                self.east_buffer.clear()
                self.down_buffer.clear()
                self.get_logger().info(
                    f'Origin locked - heading: {math.degrees(self.drone_heading):.1f}°, '
                    f'offset: [{self.offset_north:.2f}, {self.offset_east:.2f}, {self.offset_down:.2f}]')

            north_raw = tag_rel_north + self.offset_north
            east_raw = tag_rel_east + self.offset_east
            down_raw = body_down + self.offset_down

        # Apply moving average filter
        self.north_buffer.append(north_raw)
        self.east_buffer.append(east_raw)
        self.down_buffer.append(down_raw)

        north = sum(self.north_buffer) / len(self.north_buffer)
        east = sum(self.east_buffer) / len(self.east_buffer)
        down = sum(self.down_buffer) / len(self.down_buffer)

        # Build VehicleOdometry message
        msg = VehicleOdometry()
        msg.timestamp = self._tf_timestamp_us
        msg.timestamp_sample = self._tf_timestamp_us

        # Position in NED frame
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = [float(north), float(east), float(down)]

        # Orientation - NaN (not providing yaw from AprilTag)
        msg.q = [float('nan'), float('nan'), float('nan'), float('nan')]

        # Velocity - NaN (let EKF2 derive from position)
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
                heading_deg += 360
            tag_info = f'Tag {tag_id}' if self.use_tag_map else 'Tag'
            self.get_logger().info(
                f'{tag_info} | Body: [{body_forward:.2f}, {body_right:.2f}] | '
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
