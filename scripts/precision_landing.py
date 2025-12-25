#!/usr/bin/env python3
"""
Precision Landing using AprilTag detection.

State machine:
1. SEARCHING: Looking for AprilTag (LED off/default)
2. DETECTED: Tag found, waiting 5 seconds (LED purple)
3. CENTERING: Moving to center over tag (LED white)
4. LANDING: Descending while staying centered (LED red)
5. LANDED: On ground (LED green briefly, then off)

Usage:
    ros2 run dexi_apriltag precision_landing.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleLocalPosition, VehicleCommand
from dexi_interfaces.srv import LEDRingColor
import time
import math
from math import cos, sin
from enum import Enum


class LandingState(Enum):
    SEARCHING = 1
    DETECTED = 2
    CENTERING = 3
    LANDING = 4
    LANDED = 5


class PrecisionLanding(Node):
    def __init__(self):
        super().__init__('precision_landing')

        # Parameters
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('target_tag_id', 0)  # 0 = any tag
        self.declare_parameter('detection_delay', 5.0)  # seconds to wait after detection
        self.declare_parameter('centering_threshold', 0.15)  # meters - when centered enough
        self.declare_parameter('centering_gain', 0.5)  # P-gain for centering
        self.declare_parameter('max_horizontal_speed', 0.5)  # m/s
        self.declare_parameter('descent_rate', 0.3)  # m/s
        self.declare_parameter('landing_altitude', 0.15)  # meters above ground to detect landing
        self.declare_parameter('final_descent_rate', 0.15)  # slower descent near ground
        self.declare_parameter('landing_velocity_threshold', 0.05)  # m/s - detect stopped

        self.tag_family = self.get_parameter('tag_family').value
        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.detection_delay = self.get_parameter('detection_delay').value
        self.centering_threshold = self.get_parameter('centering_threshold').value
        self.centering_gain = self.get_parameter('centering_gain').value
        self.max_horizontal_speed = self.get_parameter('max_horizontal_speed').value
        self.descent_rate = self.get_parameter('descent_rate').value
        self.landing_altitude = self.get_parameter('landing_altitude').value
        self.final_descent_rate = self.get_parameter('final_descent_rate').value
        self.landing_velocity_threshold = self.get_parameter('landing_velocity_threshold').value

        # QoS for PX4
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # TF2 for AprilTag position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, px4_qos)

        # LED service client
        self.led_client = self.create_client(
            LEDRingColor, '/dexi/led_service/set_led_ring_color')

        # State
        self.state = LandingState.SEARCHING
        self.detection_time = None
        self.current_altitude = 0.0
        self.current_vz = 0.0  # Vertical velocity
        self.drone_heading = 0.0  # Heading in radians
        self.drone_x = 0.0  # Current position NED X (North)
        self.drone_y = 0.0  # Current position NED Y (East)
        self.drone_z = 0.0  # Current position NED Z (Down)
        self.tag_visible = False
        self.tag_x = 0.0  # Tag position relative to drone (forward)
        self.tag_y = 0.0  # Tag position relative to drone (right)
        self.tag_z = 0.0  # Tag distance (down)
        self.last_tag_update = None  # Time when tag position last changed
        self.last_tag_x = None
        self.last_tag_y = None
        self.ground_contact_time = None  # Time when we first detected ground contact

        # Control loop timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Offboard heartbeat (must send at >= 2Hz to stay in offboard mode)
        self.heartbeat_timer = self.create_timer(0.1, self.send_heartbeat)

        self.get_logger().info('Precision Landing initialized')
        self.get_logger().info(f'Looking for tag: {self.tag_family}:{self.target_tag_id}')
        self.get_logger().info('Waiting for tag detection...')

    def local_position_callback(self, msg):
        """Track current position, altitude, velocity, and heading."""
        self.drone_x = msg.x  # NED North
        self.drone_y = msg.y  # NED East
        self.drone_z = msg.z  # NED Down
        self.current_altitude = -msg.z  # Convert NED to altitude
        self.current_vz = msg.vz  # Vertical velocity (positive = down in NED)
        self.drone_heading = msg.heading  # Heading in radians (0 = North, positive = clockwise)

    def get_tag_position(self):
        """Get AprilTag position from TF. Returns True if tag found."""
        tag_frame = f'{self.tag_family}:{self.target_tag_id}'
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', tag_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))

            # Tag position relative to drone in body frame
            # TF frame mapping (empirically determined):
            # - TF X = altitude (distance down to tag)
            # - TF Y = south offset (opposite of forward in body frame)
            # - TF Z = west offset (opposite of right in body frame)
            new_tag_x = -transform.transform.translation.y  # Forward = -TF.y
            new_tag_y = -transform.transform.translation.z  # Right = -TF.z
            new_tag_z = transform.transform.translation.x   # Altitude = TF.x

            # Check if the position actually changed (detect stale TF by unchanged values)
            if self.last_tag_x is not None:
                dx = abs(new_tag_x - self.last_tag_x)
                dy = abs(new_tag_y - self.last_tag_y)
                if dx < 0.001 and dy < 0.001:
                    # Position hasn't changed - check if it's been too long
                    if self.last_tag_update is not None:
                        age = time.time() - self.last_tag_update
                        if age > 0.5:
                            self.get_logger().warn(f'TF stale: position unchanged for {age:.2f}s', throttle_duration_sec=1.0)
                            self.tag_visible = False
                            return False
                else:
                    # Position changed - update timestamp
                    self.last_tag_update = time.time()

            self.tag_x = new_tag_x
            self.tag_y = new_tag_y
            self.tag_z = new_tag_z
            self.last_tag_x = new_tag_x
            self.last_tag_y = new_tag_y

            if self.last_tag_update is None:
                self.last_tag_update = time.time()

            self.tag_visible = True
            return True
        except Exception as e:
            self.tag_visible = False
            return False

    def set_led_color(self, color):
        """Set LED ring color asynchronously."""
        if not self.led_client.wait_for_service(timeout_sec=0.1):
            return
        request = LEDRingColor.Request()
        request.color = color
        self.led_client.call_async(request)

    def send_heartbeat(self):
        """Send offboard control mode heartbeat."""
        msg = OffboardControlMode()
        msg.timestamp = int(time.time() * 1e6)
        msg.position = True  # Using position control like apriltag_follower
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)

    def body_to_ned(self, vx_body, vy_body):
        """Convert body-frame velocities to NED world-frame.

        Body frame: vx = forward, vy = right (FRD convention)
        NED frame: vx = north, vy = east
        """
        cos_h = cos(self.drone_heading)
        sin_h = sin(self.drone_heading)

        # Standard 2D rotation from body to world
        # Note: PX4 uses FRD body frame, so vy needs to be negated for correct rotation
        vx_ned = vx_body * cos_h - vy_body * sin_h
        vy_ned = vx_body * sin_h + vy_body * cos_h

        return vx_ned, vy_ned

    def send_position_setpoint(self, vx_body, vy_body, vz):
        """Send position setpoint based on desired body-frame velocities.

        Uses position control like apriltag_follower - computes target position
        from current position + velocity * dt.
        """
        msg = TrajectorySetpoint()
        msg.timestamp = int(time.time() * 1e6)

        # Convert body-frame velocities to NED world-frame
        vx_ned, vy_ned = self.body_to_ned(vx_body, vy_body)

        # Compute target position (current + velocity * dt)
        dt = 0.2  # Time horizon for position command (same as apriltag_follower)
        target_x = self.drone_x + vx_ned * dt
        target_y = self.drone_y + vy_ned * dt
        target_z = self.drone_z + vz * dt  # vz is already in NED (positive = down)

        # Position setpoint in NED
        msg.position = [target_x, target_y, target_z]

        # Velocity hint (optional, helps with smooth motion)
        msg.velocity = [vx_ned, vy_ned, vz]

        # No acceleration control
        msg.acceleration = [float('nan'), float('nan'), float('nan')]

        # Maintain current yaw
        msg.yaw = float('nan')
        msg.yawspeed = 0.0

        self.setpoint_pub.publish(msg)

    def send_disarm_command(self):
        """Send disarm command to PX4."""
        msg = VehicleCommand()
        msg.timestamp = int(time.time() * 1e6)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0  # 0 = disarm
        msg.param2 = 21196.0  # Force disarm (magic number)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('Disarm command sent')

    def control_loop(self):
        """Main control loop - runs at 20Hz."""
        tag_found = self.get_tag_position()

        if self.state == LandingState.SEARCHING:
            if tag_found:
                self.get_logger().info(f'Tag detected! Waiting {self.detection_delay}s before landing...')
                self.set_led_color('purple')
                self.detection_time = time.time()
                self.state = LandingState.DETECTED
            else:
                # Hold position (zero velocity)
                self.send_position_setpoint(0.0, 0.0, 0.0)

        elif self.state == LandingState.DETECTED:
            if not tag_found:
                # Lost tag, go back to searching
                self.get_logger().warn('Tag lost during detection delay, returning to search')
                self.set_led_color('black')
                self.state = LandingState.SEARCHING
            elif time.time() - self.detection_time >= self.detection_delay:
                # Delay complete, start centering
                self.get_logger().info('Starting centering maneuver')
                self.set_led_color('white')
                self.state = LandingState.CENTERING
            else:
                # Still waiting, hold position
                remaining = self.detection_delay - (time.time() - self.detection_time)
                self.get_logger().info(f'Centering in {remaining:.1f}s...', throttle_duration_sec=1.0)
                self.send_position_setpoint(0.0, 0.0, 0.0)

        elif self.state == LandingState.CENTERING:
            if not tag_found:
                # Lost tag during centering
                self.get_logger().warn('Tag lost during centering! Holding position...')
                self.send_position_setpoint(0.0, 0.0, 0.0)
            else:
                # Calculate velocity to center over tag
                # tag_x is forward offset, tag_y is right offset
                error_x = self.tag_x
                error_y = self.tag_y

                # Proportional control - move TOWARD the tag
                vx = max(-self.max_horizontal_speed,
                        min(self.max_horizontal_speed, self.centering_gain * error_x))
                vy = max(-self.max_horizontal_speed,
                        min(self.max_horizontal_speed, self.centering_gain * error_y))

                # Check if centered enough
                error_magnitude = math.sqrt(error_x**2 + error_y**2)

                # Calculate NED velocities for logging
                vx_ned, vy_ned = self.body_to_ned(vx, vy)
                self.get_logger().info(
                    f'Centering: offset=[{error_x:.2f}, {error_y:.2f}]m, err={error_magnitude:.2f}m, pos=[{self.drone_x:.2f}, {self.drone_y:.2f}]',
                    throttle_duration_sec=0.5)

                if error_magnitude < self.centering_threshold:
                    self.get_logger().info('Centered! Beginning descent...')
                    self.set_led_color('red')
                    self.state = LandingState.LANDING
                else:
                    # Continue centering (no vertical movement)
                    self.send_position_setpoint(vx, vy, 0.0)

        elif self.state == LandingState.LANDING:
            # Calculate centering velocities (even if tag lost, we use last known)
            if tag_found:
                error_x = self.tag_x
                error_y = self.tag_y
            else:
                # Tag lost - continue with zero horizontal velocity
                error_x = 0.0
                error_y = 0.0
                self.get_logger().warn('Tag lost during landing, descending straight...',
                                      throttle_duration_sec=1.0)

            vx = max(-self.max_horizontal_speed,
                    min(self.max_horizontal_speed, self.centering_gain * error_x))
            vy = max(-self.max_horizontal_speed,
                    min(self.max_horizontal_speed, self.centering_gain * error_y))

            # Use slower descent rate when close to ground
            if self.current_altitude < 0.5:
                descent = self.final_descent_rate
            else:
                descent = self.descent_rate

            # Calculate NED velocities for logging
            vx_ned, vy_ned = self.body_to_ned(vx, vy)
            self.get_logger().info(
                f'Landing: alt={self.current_altitude:.2f}m, offset=[{error_x:.2f}, {error_y:.2f}]m, pos=[{self.drone_x:.2f}, {self.drone_y:.2f}]',
                throttle_duration_sec=0.5)

            # Send position setpoint - keep centering while descending
            self.send_position_setpoint(vx, vy, descent)

            # Detect ground contact: low altitude AND velocity near zero
            if self.current_altitude < self.landing_altitude and abs(self.current_vz) < self.landing_velocity_threshold:
                if self.ground_contact_time is None:
                    self.ground_contact_time = time.time()
                    self.get_logger().info('Ground contact detected, confirming...')
                elif time.time() - self.ground_contact_time > 0.5:  # Confirm for 0.5s
                    self.get_logger().info('Landing confirmed! Disarming...')
                    self.set_led_color('green')
                    self.send_disarm_command()
                    self.state = LandingState.LANDED
            else:
                self.ground_contact_time = None  # Reset if conditions not met

        elif self.state == LandingState.LANDED:
            # Check if drone has taken off again (altitude > 1m)
            if self.current_altitude > 1.0:
                self.get_logger().info('Takeoff detected! Resetting to search mode...')
                self.set_led_color('black')
                # Reset state variables
                self.detection_time = None
                self.ground_contact_time = None
                self.last_tag_update = None
                self.last_tag_x = None
                self.last_tag_y = None
                self.state = LandingState.SEARCHING
            else:
                self.get_logger().info('Landed and disarmed!', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLanding()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Clean exit on Ctrl+C
    finally:
        # Safely clean up - context may already be invalid
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
