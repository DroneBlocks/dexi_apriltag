#!/usr/bin/env python3
"""
Precision Landing using AprilTag detection.

State machine:
1. SEARCHING: Looking for AprilTag (LED off/default)
2. DETECTED: Tag found, waiting 5 seconds (LED purple)
3. CENTERING: Slow movement (10cm/s) with filtering (LED white)
   - Requires stable lock for 1.5s before descent
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
from std_msgs.msg import Bool
from dexi_interfaces.srv import LEDRingColor
import time
import math
from math import cos, sin
from enum import Enum
from collections import deque


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
        self.declare_parameter('centering_threshold', 0.25)  # meters - when centered enough
        self.declare_parameter('stable_centering_duration', 1.5)  # seconds - must stay centered before descent
        self.declare_parameter('centering_speed', 0.10)  # m/s - slow for indoor (10cm/s)
        self.declare_parameter('landing_centering_speed', 0.20)  # m/s - faster during descent
        self.declare_parameter('filter_length', 5)  # Moving average filter samples
        self.declare_parameter('descent_rate', 0.3)  # m/s
        self.declare_parameter('landing_altitude', 0.15)  # meters above ground to detect landing
        self.declare_parameter('final_descent_rate', 0.15)  # slower descent near ground
        self.declare_parameter('landing_velocity_threshold', 0.05)  # m/s - detect stopped

        self.tag_family = self.get_parameter('tag_family').value
        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.detection_delay = self.get_parameter('detection_delay').value
        self.centering_threshold = self.get_parameter('centering_threshold').value
        self.stable_centering_duration = self.get_parameter('stable_centering_duration').value
        self.centering_speed = self.get_parameter('centering_speed').value
        self.landing_centering_speed = self.get_parameter('landing_centering_speed').value
        self.filter_length = self.get_parameter('filter_length').value
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
        self.pause_setpoints_pub = self.create_publisher(
            Bool, '/dexi/pause_setpoints', 10)

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
        self.last_position_time = None  # Track position freshness
        self.tag_visible = False
        self.tag_x = 0.0  # Tag position relative to drone (forward)
        self.tag_y = 0.0  # Tag position relative to drone (right)
        self.tag_z = 0.0  # Tag distance (down)
        self.last_tag_update = None  # Time when tag position last changed
        self.last_tag_x = None
        self.last_tag_y = None
        self.ground_contact_time = None  # Time when we first detected ground contact
        self.centered_since = None  # Time when drone first became centered (for stable lock)

        # Moving average filter buffers (ModalAI approach)
        self.tag_x_buffer = deque(maxlen=self.filter_length)
        self.tag_y_buffer = deque(maxlen=self.filter_length)

        # Control loop timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Offboard heartbeat (must send at >= 2Hz to stay in offboard mode)
        self.heartbeat_timer = self.create_timer(0.1, self.send_heartbeat)

        self.get_logger().info('Precision Landing initialized')
        self.get_logger().info(f'Tag: {self.tag_family}:{self.target_tag_id}')
        self.get_logger().info(f'Centering: threshold={self.centering_threshold}m, speed={self.centering_speed}m/s, landing_speed={self.landing_centering_speed}m/s, lock={self.stable_centering_duration}s')
        self.get_logger().info(f'Landing: descent={self.descent_rate}m/s, final={self.final_descent_rate}m/s, detect_alt={self.landing_altitude}m')
        self.get_logger().info(f'Filter: {self.filter_length} samples, detection_delay={self.detection_delay}s')
        self.get_logger().info('Waiting for tag detection...')

    def pause_offboard_setpoints(self, pause: bool, log: bool = False):
        """Pause or resume offboard manager setpoint publishing."""
        msg = Bool()
        msg.data = pause
        self.pause_setpoints_pub.publish(msg)
        if log:
            if pause:
                self.get_logger().info('Paused offboard manager setpoints')
            else:
                self.get_logger().info('Resumed offboard manager setpoints')

    def local_position_callback(self, msg):
        """Track current position, altitude, velocity, and heading."""
        self.drone_x = msg.x  # NED North
        self.drone_y = msg.y  # NED East
        self.drone_z = msg.z  # NED Down
        self.current_altitude = -msg.z  # Convert NED to altitude
        self.current_vz = msg.vz  # Vertical velocity (positive = down in NED)
        self.drone_heading = msg.heading  # Heading in radians (0 = North, positive = clockwise)
        self.last_position_time = time.time()

    def get_filtered_tag_position(self):
        """Get moving average of tag position. Returns (x, y) or (None, None) if no data."""
        if len(self.tag_x_buffer) == 0:
            return None, None
        return sum(self.tag_x_buffer) / len(self.tag_x_buffer), \
               sum(self.tag_y_buffer) / len(self.tag_y_buffer)

    def get_tag_position(self):
        """Get AprilTag position from TF. Returns True if tag found."""
        tag_frame = f'{self.tag_family}:{self.target_tag_id}'
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', tag_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02))  # 20ms timeout to not block control loop

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

            # Add to moving average buffers (ModalAI approach)
            self.tag_x_buffer.append(new_tag_x)
            self.tag_y_buffer.append(new_tag_y)

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
        msg.position = True  # Primary: position control
        msg.velocity = True  # Fallback: velocity control when position is stale
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

    def send_hold_position(self, target_x, target_y, target_z):
        """Hold at a specific NED position."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(time.time() * 1e6)
        msg.position = [target_x, target_y, target_z]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = 0.0
        self.setpoint_pub.publish(msg)

    def send_position_setpoint(self, vx_body, vy_body, vz):
        """Send position setpoint based on desired body-frame velocities.

        Uses position control like apriltag_follower - computes target position
        from current position + velocity * dt.
        """
        msg = TrajectorySetpoint()
        msg.timestamp = int(time.time() * 1e6)

        # Check if position data is fresh (< 200ms old)
        position_stale = False
        if self.last_position_time is not None:
            pos_age = time.time() - self.last_position_time
            if pos_age > 0.2:
                position_stale = True
                self.get_logger().warn(f'Position data stale ({pos_age:.2f}s)! Using velocity-only mode.',
                                      throttle_duration_sec=1.0)

        # Convert body-frame velocities to NED world-frame
        vx_ned, vy_ned = self.body_to_ned(vx_body, vy_body)

        if position_stale:
            # Position data is stale - use velocity-only control
            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.velocity = [vx_ned, vy_ned, vz]
        else:
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

        # Debug: log actual setpoint values
        if self.state == LandingState.LANDING:
            self.get_logger().debug(
                f'Setpoint: pos=[{msg.position[0]:.2f},{msg.position[1]:.2f},{msg.position[2]:.2f}] '
                f'vel=[{msg.velocity[0]:.2f},{msg.velocity[1]:.2f},{msg.velocity[2]:.2f}]')

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
                self.pause_offboard_setpoints(True, log=True)  # Take control from offboard manager
                self.state = LandingState.CENTERING
            else:
                # Still waiting, hold position
                remaining = self.detection_delay - (time.time() - self.detection_time)
                self.get_logger().info(f'Centering in {remaining:.1f}s...', throttle_duration_sec=1.0)
                self.send_position_setpoint(0.0, 0.0, 0.0)

        elif self.state == LandingState.CENTERING:
            # Re-assert pause every loop to handle message loss
            self.pause_offboard_setpoints(True)

            # Very slow continuous movement toward filtered tag position
            if not tag_found:
                self.get_logger().warn('Tag lost! Holding position...', throttle_duration_sec=1.0)
                self.send_hold_position(self.drone_x, self.drone_y, self.drone_z)
                self.centered_since = None
                return

            # Use RAW position for threshold check (prevents false "centered" during orbiting)
            # The moving average can make it appear centered when averaging positions on both sides
            raw_magnitude = math.sqrt(self.tag_x**2 + self.tag_y**2)

            # Use filtered position for velocity commands (smoother motion)
            filtered_x, filtered_y = self.get_filtered_tag_position()
            error_x = filtered_x if filtered_x is not None else self.tag_x
            error_y = filtered_y if filtered_y is not None else self.tag_y

            if raw_magnitude < self.centering_threshold:
                # Centered - track stable duration
                if self.centered_since is None:
                    self.centered_since = time.time()
                    self.get_logger().info(f'Centered! Holding for {self.stable_centering_duration:.1f}s...')

                stable_time = time.time() - self.centered_since
                if stable_time >= self.stable_centering_duration:
                    self.get_logger().info('Stable lock confirmed! Beginning descent...')
                    self.set_led_color('red')
                    self.state = LandingState.LANDING
                else:
                    remaining = self.stable_centering_duration - stable_time
                    self.get_logger().info(
                        f'Holding: raw={raw_magnitude:.2f}m, lock in {remaining:.1f}s',
                        throttle_duration_sec=0.5)
                    # Hold current position
                    self.send_hold_position(self.drone_x, self.drone_y, self.drone_z)
            else:
                # Not centered - move very slowly toward tag
                self.centered_since = None

                # Normalize direction and apply fixed slow speed
                filtered_magnitude = math.sqrt(error_x**2 + error_y**2)
                if filtered_magnitude > 0.01:
                    vx = (error_x / filtered_magnitude) * self.centering_speed
                    vy = (error_y / filtered_magnitude) * self.centering_speed
                else:
                    vx = 0.0
                    vy = 0.0

                self.get_logger().info(
                    f'Centering: raw={raw_magnitude:.2f}m, filtered={filtered_magnitude:.2f}m, pos=[{self.tag_x:.2f}, {self.tag_y:.2f}]',
                    throttle_duration_sec=0.5)

                # Move slowly toward tag
                self.send_position_setpoint(vx, vy, 0.0)

        elif self.state == LandingState.LANDING:
            # Re-assert pause to ensure offboard manager doesn't interfere
            # (handles case where pause message was lost or offboard manager restarted)
            self.pause_offboard_setpoints(True)

            # Debug: log position freshness
            if self.last_position_time is not None:
                pos_age_ms = (time.time() - self.last_position_time) * 1000
                if pos_age_ms > 100:
                    self.get_logger().warn(f'Position age: {pos_age_ms:.0f}ms', throttle_duration_sec=0.5)

            # During landing, descend while trying to stay centered
            # Use FASTER centering speed during descent (perspective changes rapidly)
            if tag_found:
                # Use raw position for offset - more responsive during descent
                error_x = self.tag_x
                error_y = self.tag_y
            else:
                # Tag lost - descend straight
                error_x = 0.0
                error_y = 0.0
                self.get_logger().warn('Tag lost during landing, descending straight...',
                                      throttle_duration_sec=1.0)

            error_magnitude = math.sqrt(error_x**2 + error_y**2)

            # Safety: if we've drifted too far from center, abort descent and re-center
            if tag_found and error_magnitude > self.centering_threshold * 2:
                self.get_logger().warn(f'Offset too large ({error_magnitude:.2f}m)! Returning to CENTERING...')
                self.set_led_color('white')
                self.state = LandingState.CENTERING
                self.centered_since = None
                return

            # Use PROPORTIONAL control during landing to prevent orbiting/overshoot
            # Speed scales linearly with distance: 0.05m/s per 0.1m of offset
            # This prevents the oscillation caused by fixed-speed control
            gain = 0.5  # m/s per meter of offset
            max_speed = self.landing_centering_speed  # Cap at max speed

            if error_magnitude > 0.02:
                speed = min(error_magnitude * gain, max_speed)
                vx = (error_x / error_magnitude) * speed
                vy = (error_y / error_magnitude) * speed
            else:
                vx = 0.0
                vy = 0.0

            # Use slower descent rate when close to ground
            if self.current_altitude < 0.5:
                descent = self.final_descent_rate
            else:
                descent = self.descent_rate

            self.get_logger().info(
                f'Landing: alt={self.current_altitude:.2f}m, offset={error_magnitude:.2f}m, vz={descent:.2f}, vel=[{vx:.2f},{vy:.2f}]',
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
                    self.pause_offboard_setpoints(False, log=True)  # Resume offboard manager
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
                self.centered_since = None
                self.tag_x_buffer.clear()
                self.tag_y_buffer.clear()
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
        # Resume offboard manager setpoints before shutting down
        try:
            node.pause_offboard_setpoints(False, log=True)
        except Exception:
            pass
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
