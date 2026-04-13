#!/usr/bin/env python3
"""
tag_yaw_track — rotate the drone to keep an AprilTag centered in the forward camera.

A minimal visual-servoing example: the drone holds position while this node reads
the pixel-x of the tracked AprilTag's center and commands a yaw rate proportional
to the horizontal error. No OpenCV required — apriltag_ros already computed the
tag center during detection.

Control pipeline (one direction: yaw only):

    tag pixel_x  →  pixel_error  →  angle_error  →  Kp  →  yaw_rate_cmd
                    (from centre)    (via HFOV)          (clamped)

The node publishes TrajectorySetpoint directly to PX4 so it works whether or not
dexi_offboard is running. If the offboard manager is present we also tell it to
pause its own setpoints via /dexi/pause_setpoints so the two controllers don't
fight each other.

Prerequisites:
  - Drone is armed and in offboard mode (takeoff via dexi_offboard or any other way)
  - apriltag_ros is publishing /apriltag_detections
  - A forward-facing camera with a known horizontal field of view
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from apriltag_msgs.msg import AprilTagDetectionArray
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleLocalPosition,
)
from std_msgs.msg import Bool


class TagYawTrack(Node):
    def __init__(self):
        super().__init__('tag_yaw_track')

        # --- Parameters ---
        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('kp_yaw', 1.5)              # rad/s of yaw rate per rad of pixel-angle error
        self.declare_parameter('deadband_pixels', 8.0)     # ignore errors smaller than this
        self.declare_parameter('max_yaw_rate', 0.6)        # rad/s hard cap on commanded yaw rate
        self.declare_parameter('image_width', 640)
        self.declare_parameter('horizontal_fov_deg', 62.2)
        self.declare_parameter('tag_timeout', 0.5)         # seconds; stop rotating if no tag seen
        self.declare_parameter('publish_rate', 20.0)       # Hz for setpoint publishing
        self.declare_parameter('pause_offboard_manager', True)

        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.deadband_pixels = self.get_parameter('deadband_pixels').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.image_width = int(self.get_parameter('image_width').value)
        self.hfov_rad = math.radians(self.get_parameter('horizontal_fov_deg').value)
        self.tag_timeout = self.get_parameter('tag_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.pause_offboard_manager = self.get_parameter('pause_offboard_manager').value

        # Precompute: radians per pixel at the center of the image
        #   A pixel_error of image_width/2 corresponds to an angle of HFOV/2
        self.rad_per_pixel = self.hfov_rad / self.image_width

        # --- State ---
        self.latest_pixel_error = 0.0      # signed; positive = tag is right of center
        self.tag_visible = False
        self.last_tag_time = self.get_clock().now()
        self.hold_x = 0.0                  # held position for trajectory setpoint
        self.hold_y = 0.0
        self.hold_z = -1.5
        self.hold_yaw = 0.0                # captured once — lets PX4 integrate yaw from rate
        self.have_position = False

        # --- QoS ---
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # --- Subscribers ---
        self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.detection_callback,
            10,
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            px4_qos,
        )

        # --- Publishers ---
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos,
        )
        self.pause_pub = self.create_publisher(
            Bool,
            '/dexi/pause_setpoints',
            10,
        )

        # --- Control timer ---
        period = 1.0 / self.publish_rate
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f'tag_yaw_track started — tracking tag {self.target_tag_id}, '
            f'Kp={self.kp_yaw}, deadband={self.deadband_pixels}px, '
            f'max rate={self.max_yaw_rate} rad/s'
        )
        self.get_logger().info(
            f'image_width={self.image_width}, HFOV={math.degrees(self.hfov_rad):.1f}°, '
            f'rad/pixel={self.rad_per_pixel:.5f}'
        )

        if self.pause_offboard_manager:
            self._pause_offboard_manager()

    # --- Callbacks ---

    def detection_callback(self, msg):
        """Find the tracked tag and compute the pixel-x error."""
        for det in msg.detections:
            if det.id != self.target_tag_id:
                continue
            cx = det.centre.x
            self.latest_pixel_error = cx - (self.image_width / 2.0)
            self.tag_visible = True
            self.last_tag_time = self.get_clock().now()
            return

    def local_position_callback(self, msg):
        """Capture the drone's current position to hold while yawing."""
        if not self.have_position:
            self.hold_x = msg.x
            self.hold_y = msg.y
            self.hold_z = msg.z
            self.hold_yaw = msg.heading
            self.have_position = True
            self.get_logger().info(
                f'Position captured: ({self.hold_x:.2f}, {self.hold_y:.2f}, {self.hold_z:.2f}), '
                f'heading={math.degrees(self.hold_yaw):.1f}°'
            )

    # --- Control loop ---

    def control_loop(self):
        """Publish offboard mode and a trajectory setpoint with the computed yaw rate."""
        # 1. Keep offboard control mode alive (heartbeat)
        mode = OffboardControlMode()
        mode.timestamp = self._get_px4_timestamp()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.offboard_mode_pub.publish(mode)

        # 2. Compute yaw rate from the latest detection
        yaw_rate = self._compute_yaw_rate()

        # 3. Publish trajectory setpoint: hold position, command yaw rate
        if self.have_position:
            sp = TrajectorySetpoint()
            sp.timestamp = self._get_px4_timestamp()
            sp.position = [float(self.hold_x), float(self.hold_y), float(self.hold_z)]
            sp.velocity = [float('nan'), float('nan'), float('nan')]
            sp.acceleration = [float('nan'), float('nan'), float('nan')]
            sp.yaw = float('nan')               # let PX4 integrate from yawspeed
            sp.yawspeed = float(yaw_rate)
            self.setpoint_pub.publish(sp)

    def _compute_yaw_rate(self):
        """Convert pixel-x error into a yaw-rate command (rad/s)."""
        # Check tag freshness
        elapsed = (self.get_clock().now() - self.last_tag_time).nanoseconds / 1e9
        if not self.tag_visible or elapsed > self.tag_timeout:
            return 0.0

        # Deadband: tiny errors → no command (avoids jitter at the center)
        if abs(self.latest_pixel_error) < self.deadband_pixels:
            return 0.0

        # Pixel error → angular error (radians)
        angle_error = self.latest_pixel_error * self.rad_per_pixel

        # Proportional control. Positive angle_error means the tag is to the right of
        # center, so we want to yaw right (positive yawspeed in NED is clockwise when
        # viewed from above, which rotates the body +X toward the tag).
        yaw_rate = self.kp_yaw * angle_error

        # Clamp to configured maximum
        if yaw_rate > self.max_yaw_rate:
            yaw_rate = self.max_yaw_rate
        elif yaw_rate < -self.max_yaw_rate:
            yaw_rate = -self.max_yaw_rate

        return yaw_rate

    def _pause_offboard_manager(self):
        """Ask dexi_offboard_manager to stop publishing its own setpoints."""
        msg = Bool()
        msg.data = True
        self.pause_pub.publish(msg)
        self.get_logger().info('Sent pause_setpoints=true (if offboard manager is running)')

    def _get_px4_timestamp(self):
        """PX4 expects microseconds since boot."""
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = TagYawTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On exit, release the offboard manager so position hold resumes
        if node.pause_offboard_manager:
            resume = Bool()
            resume.data = False
            node.pause_pub.publish(resume)
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
