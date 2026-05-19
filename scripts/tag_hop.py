#!/usr/bin/env python3
"""
Tag-hop corridor navigator.

Drone flies a sequence of AprilTag waypoints (e.g. [0, 2, 4, 2, 0]):
takeoff over the first tag → center → yaw-align to tag's up-edge → hold
→ body-frame velocity transit to next tag → repeat → AUTO.LAND when
sequence finishes.

Why body-frame velocity instead of NED setpoints: PX4's EKF NED drifts on
flow + IMU, so NED-target setpoints fight a moving estimate. Body-frame
velocity uses gyro+accel attitude (very accurate) — drone moves in real
world regardless of EKF position state. Vision (TF for the next tag in
the sequence) decides arrival, not EKF distance.

Yaw alignment (Approach A): on first centering, the drone reads the tag's
own rotation from TF and rotates body-forward to match the tag's printed
up-edge. Closed-loop: target_yaw recomputes each cycle from the visible
tag, so PX4 controller noise self-corrects. Robust to EKF heading bias —
the correction is purely body-relative (what the camera sees).

LED sequence: off → purple (settle) → cyan (centering) → white (locked
in HOLDING) → yellow (transit) → red (descent) → green (landed).

Manual override: any RC mode flip away from OFFBOARD triggers a sticky
abort. Land manually, take off again, sequence re-engages automatically.
"""

import math
import time
from collections import deque
from enum import Enum
from math import cos, sin

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from dexi_interfaces.srv import LEDRingColor
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)


class HopState(Enum):
    SEARCHING = 1     # waiting for first detection of starting tag
    DETECTED = 2      # tag found, 2 s settle window
    CENTERING = 3     # active centering on current target tag
    HOLDING = 4       # hover at locked tag NED for hover_duration
    TRANSIT = 5       # flying toward next tag at slow velocity
    AUTO_LANDING = 6  # PX4 AUTO.LAND handles descent + disarm
    LANDED = 7        # disarmed


class TagHop(Node):
    def __init__(self):
        super().__init__('tag_hop')

        # ----- Parameters -----
        self.declare_parameter('tag_family', 'tag36h11')
        # Sequence of tag IDs to visit, in order. Default: 0 -> 1 -> 0 -> land.
        self.declare_parameter('sequence', [0, 1, 0])
        # Tag map (parallel arrays): tag_ids -> NED (n, e). Used as transit targets.
        self.declare_parameter('tag_map_ids', [0, 1])
        self.declare_parameter('tag_map_n', [0.0, 1.0])
        self.declare_parameter('tag_map_e', [0.0, 0.0])
        self.declare_parameter('hover_duration', 10.0)        # s at each tag
        self.declare_parameter('transit_speed', 0.20)         # m/s during TRANSIT
        self.declare_parameter('centering_speed', 0.20)       # m/s while chasing a tag in CENTERING
        self.declare_parameter('min_transit_duration', 1.0)   # s minimum body-frame TRANSIT before allowing tag acquisition
        self.declare_parameter('centering_threshold', 0.25)   # m raw_magnitude to declare centered
        self.declare_parameter('centered_exit_threshold', 0.40)  # m hysteresis exit
        self.declare_parameter('transit_timeout', 15.0)       # s max TRANSIT duration before aborting to AUTO.LAND
        self.declare_parameter('detection_delay', 2.0)        # s settle after first detection
        self.declare_parameter('filter_length', 5)
        self.declare_parameter('min_takeoff_altitude', 0.30)
        self.declare_parameter('tag_loss_grace', 2.0)         # s tag-loss before timer reset
        self.declare_parameter('detection_led_color', 'cyan') # LED color during CENTERING (acquiring/chasing the tag)
        self.declare_parameter('lock_led_color', 'white')     # LED color during HOLDING (locked on tag)
        self.declare_parameter('yaw_rate_max', 0.15)          # rad/s slew limit during yaw alignment

        self.tag_family = self.get_parameter('tag_family').value
        self.sequence = list(self.get_parameter('sequence').value)
        tag_map_ids = list(self.get_parameter('tag_map_ids').value)
        tag_map_n = list(self.get_parameter('tag_map_n').value)
        tag_map_e = list(self.get_parameter('tag_map_e').value)
        if not (len(tag_map_ids) == len(tag_map_n) == len(tag_map_e)):
            raise RuntimeError('tag_map_ids/n/e must be same length')
        self.tag_map = {tid: (n, e) for tid, n, e in zip(tag_map_ids, tag_map_n, tag_map_e)}
        for tid in self.sequence:
            if tid not in self.tag_map:
                raise RuntimeError(f'sequence references tag {tid} not in tag_map')

        self.hover_duration = self.get_parameter('hover_duration').value
        self.transit_speed = self.get_parameter('transit_speed').value
        self.centering_speed = self.get_parameter('centering_speed').value
        self.min_transit_duration = self.get_parameter('min_transit_duration').value
        self.centering_threshold = self.get_parameter('centering_threshold').value
        self.centered_exit_threshold = self.get_parameter('centered_exit_threshold').value
        self.transit_timeout = self.get_parameter('transit_timeout').value
        self.detection_delay = self.get_parameter('detection_delay').value
        self.filter_length = self.get_parameter('filter_length').value
        self.min_takeoff_altitude = self.get_parameter('min_takeoff_altitude').value
        self.tag_loss_grace = self.get_parameter('tag_loss_grace').value
        self.detection_led_color = self.get_parameter('detection_led_color').value
        self.lock_led_color = self.get_parameter('lock_led_color').value
        self.yaw_rate_max = self.get_parameter('yaw_rate_max').value

        # ----- ROS -----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.pause_setpoints_pub = self.create_publisher(
            Bool, '/dexi/pause_setpoints', 10)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, px4_qos)
        self.land_detected = False
        self.land_detected_sub = self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self.land_detected_callback, px4_qos)
        self.nav_state = None
        self.was_in_offboard = False
        # True only after WE send the OFFBOARD command. Without this gate,
        # if the FC is still reporting OFFBOARD from a previous flight (e.g.
        # the prior tag_hop crashed mid-flight), this script's vehicle_status
        # callback would set was_in_offboard=True before we ever engaged,
        # and the FC's subsequent timeout to a non-OFFBOARD mode would trip
        # the manual-override path before takeoff.
        self.offboard_requested = False
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, px4_qos)

        self.led_client = self.create_client(
            LEDRingColor, '/dexi/led_service/set_led_ring_color')

        # ----- State -----
        self.state = HopState.SEARCHING
        self.sequence_index = 0
        self.detection_time = None
        self.centered_since = None
        self.hold_start_time = None
        self.transit_start_time = None
        self.target_x = None
        self.target_y = None
        self.target_z = None  # locked altitude, captured on first CENTERING
        self.tag_loss_start = None
        self.aborted = False  # set on manual override; sticks until restart

        # Drone state
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_heading = 0.0
        self.current_altitude = 0.0
        self.current_vz = 0.0
        self.last_position_time = None

        # Latest tag pose (body frame) for the CURRENT target tag
        self.tag_x = 0.0   # forward
        self.tag_y = 0.0   # right
        self.tag_z = 0.0   # down
        self.tag_visible = False
        self.last_tag_x = None
        self.last_tag_y = None
        self.last_tag_update = None
        self.tag_x_buffer = deque(maxlen=self.filter_length)
        self.tag_y_buffer = deque(maxlen=self.filter_length)

        # Yaw alignment state. target_yaw is the live setpoint (recomputed
        # from the visible tag every CENTERING cycle); commanded_yaw is the
        # slew-limited intermediate that PX4 sees, so the controller never
        # gets a step input.
        self.target_yaw = None
        self.commanded_yaw = None
        self.yaw_tolerance = math.radians(5.0)

        # ----- Timers -----
        self.control_timer = self.create_timer(0.05, self.control_loop)   # 20 Hz
        self.heartbeat_timer = self.create_timer(0.1, self.send_heartbeat)  # 10 Hz

        self.get_logger().info('Tag Hop initialized')
        self.get_logger().info(f'Sequence: {self.sequence}')
        self.get_logger().info(f'Tag map: {self.tag_map}')
        self.get_logger().info(f'Hover {self.hover_duration}s, transit {self.transit_speed} m/s')
        self.get_logger().info('Waiting for first detection of tag '
                               f'{self.sequence[0]} above {self.min_takeoff_altitude}m...')

    # ----- Callbacks -----

    def local_position_callback(self, msg):
        self.drone_x = msg.x
        self.drone_y = msg.y
        self.drone_z = msg.z
        self.current_altitude = -msg.z
        self.current_vz = msg.vz
        self.drone_heading = msg.heading
        self.last_position_time = time.time()

    def land_detected_callback(self, msg):
        if msg.landed and not self.land_detected:
            self.get_logger().info('PX4 reports vehicle_land_detected.landed=True')
        self.land_detected = msg.landed

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        if (self.offboard_requested
                and msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.was_in_offboard = True

    # ----- Helpers -----

    def current_target_tag_id(self):
        if self.sequence_index < len(self.sequence):
            return self.sequence[self.sequence_index]
        return None

    def transit_direction_body(self):
        """Body-frame unit vector to fly during the current TRANSIT leg.

        Computed from the tag_map delta (prev tag -> next tag). The yaw
        alignment in CENTERING ensures body-forward is aligned with the
        tag's up-edge, so the tag_map +N direction maps directly onto
        body-forward as long as the user lays tags with up-edges along
        the same direction the tag_map encodes.
        """
        if self.sequence_index <= 0 or self.sequence_index >= len(self.sequence):
            return 0.0, 0.0
        prev_tag = self.sequence[self.sequence_index - 1]
        next_tag = self.sequence[self.sequence_index]
        prev_n, prev_e = self.tag_map[prev_tag]
        next_n, next_e = self.tag_map[next_tag]
        delta_n = next_n - prev_n  # body-forward component
        delta_e = next_e - prev_e  # body-right component
        mag = math.sqrt(delta_n ** 2 + delta_e ** 2)
        if mag < 1e-6:
            return 0.0, 0.0
        return delta_n / mag, delta_e / mag

    def compute_target_yaw_from_tag(self):
        """NED yaw to align body-forward with the visible tag's up-edge.

        Returns (target_yaw, beta) where beta is the body-frame angle from
        forward to tag-up. drone_heading + beta gives the absolute setpoint;
        the addition cancels any constant EKF heading bias because beta is
        purely body-relative.
        """
        tid = self.current_target_tag_id()
        if tid is None:
            return None, None
        tag_frame = f'{self.tag_family}:{tid}'
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', tag_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:
            return None, None
        q = t.transform.rotation
        # Rotate tag's local +Y axis (the up-edge) by q to express it in
        # base_link's TF-native frame, then remap to body FRD using the
        # same convention as get_tag_position(): body_fwd = -v_y,
        # body_right = -v_z.
        v_y = 1 - 2 * (q.x * q.x + q.z * q.z)
        v_z = 2 * (q.y * q.z + q.w * q.x)
        beta = math.atan2(-v_z, -v_y)
        target = self.drone_heading + beta
        while target > math.pi:
            target -= 2 * math.pi
        while target < -math.pi:
            target += 2 * math.pi
        return target, beta

    def yaw_error(self):
        if self.target_yaw is None:
            return 0.0
        diff = self.target_yaw - self.drone_heading
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def advance_commanded_yaw(self, advance=True, dt=0.05):
        """Slew-limited yaw setpoint, advanced once per setpoint publish.
        advance=False freezes it — used during tag loss so the drone
        doesn't keep rotating without visual confirmation."""
        if self.target_yaw is None or self.commanded_yaw is None:
            return None
        if not advance:
            return self.commanded_yaw
        diff = self.target_yaw - self.commanded_yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        max_step = self.yaw_rate_max * dt
        if abs(diff) > max_step:
            self.commanded_yaw += math.copysign(max_step, diff)
        else:
            self.commanded_yaw = self.target_yaw
        while self.commanded_yaw > math.pi:
            self.commanded_yaw -= 2 * math.pi
        while self.commanded_yaw < -math.pi:
            self.commanded_yaw += 2 * math.pi
        return self.commanded_yaw

    def get_tag_position(self):
        """TF lookup for the CURRENT target tag. Sets self.tag_x/y/z (body frame)."""
        tid = self.current_target_tag_id()
        if tid is None:
            return False
        tag_frame = f'{self.tag_family}:{tid}'
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', tag_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02))
            new_tag_x = -t.transform.translation.y   # forward
            new_tag_y = -t.transform.translation.z   # right
            new_tag_z = t.transform.translation.x    # down
            # Stale-TF detection: if values haven't changed for > 0.5 s, treat as lost
            if self.last_tag_x is not None:
                dx = abs(new_tag_x - self.last_tag_x)
                dy = abs(new_tag_y - self.last_tag_y)
                if dx < 0.001 and dy < 0.001:
                    if self.last_tag_update is not None:
                        age = time.time() - self.last_tag_update
                        if age > 0.5:
                            self.tag_visible = False
                            return False
                else:
                    self.last_tag_update = time.time()
            self.tag_x = new_tag_x
            self.tag_y = new_tag_y
            self.tag_z = new_tag_z
            self.last_tag_x = new_tag_x
            self.last_tag_y = new_tag_y
            self.tag_x_buffer.append(new_tag_x)
            self.tag_y_buffer.append(new_tag_y)
            if self.last_tag_update is None:
                self.last_tag_update = time.time()
            self.tag_visible = True
            return True
        except Exception:
            self.tag_visible = False
            return False

    def reset_tag_filter(self):
        """Clear filter buffers when switching to a different target tag."""
        self.tag_x_buffer.clear()
        self.tag_y_buffer.clear()
        self.last_tag_x = None
        self.last_tag_y = None
        self.last_tag_update = None
        self.tag_visible = False

    def get_filtered_tag_position(self):
        if len(self.tag_x_buffer) == 0:
            return None, None
        return (sum(self.tag_x_buffer) / len(self.tag_x_buffer),
                sum(self.tag_y_buffer) / len(self.tag_y_buffer))

    def body_to_ned(self, vx_body, vy_body):
        c = cos(self.drone_heading)
        s = sin(self.drone_heading)
        return vx_body * c - vy_body * s, vx_body * s + vy_body * c

    def set_led_color(self, color):
        if not self.led_client.wait_for_service(timeout_sec=0.1):
            return
        request = LEDRingColor.Request()
        request.color = color
        self.led_client.call_async(request)

    def pause_offboard_setpoints(self, pause: bool, log: bool = False):
        msg = Bool()
        msg.data = pause
        self.pause_setpoints_pub.publish(msg)
        if log:
            self.get_logger().info(f'pause_offboard_setpoints({pause})')

    # ----- Setpoint helpers -----

    def send_heartbeat(self):
        # Stop advertising OFFBOARD once we've handed off to PX4 AUTO.LAND.
        # Otherwise the heartbeat keeps signaling "OFFBOARD wants control"
        # which can interfere with PX4's descent.
        if self.state == HopState.AUTO_LANDING or self.aborted:
            return
        msg = OffboardControlMode()
        msg.timestamp = int(time.time() * 1e6)
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)
        self.pause_offboard_setpoints(True)

    def send_hold_position(self, target_x, target_y, target_z, advance_yaw=True):
        msg = TrajectorySetpoint()
        msg.timestamp = int(time.time() * 1e6)
        msg.position = [float(target_x), float(target_y), float(target_z)]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        yaw_cmd = self.advance_commanded_yaw(advance=advance_yaw)
        msg.yaw = float(yaw_cmd) if yaw_cmd is not None else float('nan')
        msg.yawspeed = 0.0
        self.setpoint_pub.publish(msg)

    def send_position_setpoint(self, vx_body, vy_body, vz, fixed_z=None):
        msg = TrajectorySetpoint()
        msg.timestamp = int(time.time() * 1e6)
        position_stale = (self.last_position_time is not None
                          and (time.time() - self.last_position_time) > 0.2)
        vx_ned, vy_ned = self.body_to_ned(vx_body, vy_body)
        if position_stale:
            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.velocity = [vx_ned, vy_ned, vz]
        else:
            dt = 0.2
            target_x = self.drone_x + vx_ned * dt
            target_y = self.drone_y + vy_ned * dt
            if fixed_z is not None:
                target_z = fixed_z
            else:
                target_z = self.drone_z + vz * dt
            msg.position = [float(target_x), float(target_y), float(target_z)]
            msg.velocity = [vx_ned, vy_ned, vz]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        yaw_cmd = self.advance_commanded_yaw()
        msg.yaw = float(yaw_cmd) if yaw_cmd is not None else float('nan')
        msg.yawspeed = 0.0
        self.setpoint_pub.publish(msg)

    def send_offboard_mode_command(self):
        msg = VehicleCommand()
        msg.timestamp = int(time.time() * 1e6)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('OFFBOARD mode command sent')

    def send_auto_land_mode_command(self):
        msg = VehicleCommand()
        msg.timestamp = int(time.time() * 1e6)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 4.0  # PX4_CUSTOM_MAIN_MODE_AUTO
        msg.param3 = 6.0  # PX4_CUSTOM_SUB_MODE_AUTO_LAND
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('AUTO.LAND mode command sent')

    def start_auto_landing(self):
        """Hand off to PX4 AUTO.LAND for descent + disarm."""
        self.set_led_color('red')
        self.send_auto_land_mode_command()
        self.state = HopState.AUTO_LANDING

    # ----- Main control loop -----

    def control_loop(self):
        # After a manual override, stay quiet until the user has manually
        # landed AND lifted off again — at which point we treat the new
        # takeoff as a fresh attempt and re-arm the state machine. Without
        # this, the user has to ssh-restart the script between flights.
        if self.aborted:
            if self.land_detected:
                self.get_logger().info(
                    'Manual landing detected after override — '
                    'ready for next flight.')
                self.aborted = False
                self.was_in_offboard = False
                self.offboard_requested = False
                self.target_yaw = None
                self.commanded_yaw = None
                self.state = HopState.LANDED
                self.set_led_color('black')
            return

        tag_found = self.get_tag_position()

        # Manual override detection: user took RC mode control.
        if (self.was_in_offboard and self.nav_state is not None
                and self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and self.state not in (HopState.LANDED, HopState.AUTO_LANDING)):
            self.get_logger().warn(
                f'Manual override (nav_state={self.nav_state}); sequence '
                f'aborted. Land and take off again to re-engage.')
            self.set_led_color('black')
            self.aborted = True
            self.pause_offboard_setpoints(False, log=True)
            return

        if self.state == HopState.SEARCHING:
            airborne = self.current_altitude > self.min_takeoff_altitude
            if tag_found and airborne:
                self.get_logger().info(
                    f'Tag {self.current_target_tag_id()} detected airborne — '
                    f'waiting {self.detection_delay}s before engaging.')
                self.set_led_color('purple')
                self.detection_time = time.time()
                self.state = HopState.DETECTED
            else:
                self.send_position_setpoint(0.0, 0.0, 0.0)

        elif self.state == HopState.DETECTED:
            if not tag_found:
                self.get_logger().warn('Tag lost during detection delay, returning to SEARCHING')
                self.set_led_color('black')
                self.state = HopState.SEARCHING
            elif time.time() - self.detection_time >= self.detection_delay:
                self.get_logger().info(
                    f'Engaging on tag {self.current_target_tag_id()} '
                    f'(LED {self.detection_led_color})')
                self.set_led_color(self.detection_led_color)
                self.pause_offboard_setpoints(True, log=True)
                self.send_offboard_mode_command()
                self.offboard_requested = True
                # Lock target altitude on first centering
                self.target_z = self.drone_z
                self.target_x = None
                self.target_y = None
                self.centered_since = None
                self.tag_loss_start = None
                self.state = HopState.CENTERING
                self.get_logger().info(
                    f'Locked target altitude: {-self.target_z:.2f} m')
            else:
                self.send_position_setpoint(0.0, 0.0, 0.0)

        elif self.state == HopState.CENTERING:
            self.pause_offboard_setpoints(True)
            if not tag_found:
                if self.tag_loss_start is None:
                    self.tag_loss_start = time.time()
                loss_duration = time.time() - self.tag_loss_start
                self.get_logger().warn(
                    f'Tag {self.current_target_tag_id()} lost ({loss_duration:.1f}s)! Holding.',
                    throttle_duration_sec=1.0)
                hx = self.target_x if self.target_x is not None else self.drone_x
                hy = self.target_y if self.target_y is not None else self.drone_y
                hz = self.target_z if self.target_z is not None else self.drone_z
                # Freeze yaw during tag loss — rotating blindly pushes the
                # tag farther from the recovery zone.
                self.send_hold_position(hx, hy, hz, advance_yaw=False)
                if loss_duration > self.tag_loss_grace:
                    self.centered_since = None
                return
            self.tag_loss_start = None

            raw_magnitude = math.sqrt(self.tag_x ** 2 + self.tag_y ** 2)
            already_centered = self.centered_since is not None
            entered = raw_magnitude < self.centering_threshold
            still_in = raw_magnitude < self.centered_exit_threshold

            if entered or (already_centered and still_in):
                if self.centered_since is None:
                    self.centered_since = time.time()
                # Lock / refine target NED at the tag's location
                tag_n_off, tag_e_off = self.body_to_ned(self.tag_x, self.tag_y)
                new_target_x = self.drone_x + tag_n_off
                new_target_y = self.drone_y + tag_e_off
                if self.target_x is None:
                    self.target_x = new_target_x
                    self.target_y = new_target_y
                    self.get_logger().info(
                        f'Centered on tag {self.current_target_tag_id()} at '
                        f'NED [{self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}]')
                    if self.target_yaw is None:
                        self.target_yaw, beta = self.compute_target_yaw_from_tag()
                        if self.target_yaw is not None:
                            self.commanded_yaw = self.drone_heading
                            self.get_logger().info(
                                f'Yaw target: {math.degrees(self.target_yaw):.1f}° '
                                f'(current: {math.degrees(self.drone_heading):.1f}°, '
                                f'error: {math.degrees(self.yaw_error()):.1f}°, '
                                f'tag-rel β: {math.degrees(beta):.1f}°, '
                                f'rate cap {math.degrees(self.yaw_rate_max):.0f}°/s)')
                else:
                    # Closed-loop: target_yaw chases the visible tag's actual
                    # up-edge each cycle, so β-noise and PX4 controller
                    # overshoot self-correct.
                    new_yaw, _ = self.compute_target_yaw_from_tag()
                    if new_yaw is not None:
                        self.target_yaw = new_yaw
                    # Snap target_x/y to live observation while yaw is still
                    # rotating, since EKF drifts under flow-only rotation.
                    # Once aligned, fall back to filtered tracking.
                    yaw_aligning = (self.target_yaw is not None
                                    and abs(self.yaw_error()) >= self.yaw_tolerance)
                    alpha = 0.0 if yaw_aligning else 0.8
                    self.target_x = alpha * self.target_x + (1.0 - alpha) * new_target_x
                    self.target_y = alpha * self.target_y + (1.0 - alpha) * new_target_y
                # HOLDING gate: TRANSIT semantics depend on a known yaw,
                # so don't enter HOLDING (and thus the next TRANSIT) until
                # yaw error is within tolerance.
                yaw_ok = (self.target_yaw is None
                          or abs(self.yaw_error()) < self.yaw_tolerance)
                if time.time() - self.centered_since >= 1.0 and yaw_ok:
                    self.get_logger().info(
                        f'Holding tag {self.current_target_tag_id()} for '
                        f'{self.hover_duration:.0f}s (LED {self.lock_led_color})')
                    self.set_led_color(self.lock_led_color)
                    self.hold_start_time = time.time()
                    self.state = HopState.HOLDING
                elif not yaw_ok:
                    self.get_logger().info(
                        f'Yaw aligning: error={math.degrees(self.yaw_error()):.1f}°',
                        throttle_duration_sec=1.0)
                self.send_hold_position(self.target_x, self.target_y, self.target_z)
            else:
                # Not centered — chase the tag at slow speed
                self.centered_since = None
                fx, fy = self.get_filtered_tag_position()
                ex = fx if fx is not None else self.tag_x
                ey = fy if fy is not None else self.tag_y
                mag = math.sqrt(ex ** 2 + ey ** 2)
                if mag > 0.01:
                    vx = (ex / mag) * self.centering_speed
                    vy = (ey / mag) * self.centering_speed
                else:
                    vx = vy = 0.0
                self.get_logger().info(
                    f'Centering on tag {self.current_target_tag_id()}: '
                    f'raw={raw_magnitude:.2f}m, vx_body={vx:.2f}, vy_body={vy:.2f}',
                    throttle_duration_sec=0.5)
                self.send_position_setpoint(vx, vy, 0.0, fixed_z=self.target_z)

        elif self.state == HopState.HOLDING:
            self.pause_offboard_setpoints(True)
            elapsed = time.time() - self.hold_start_time
            # Continuously refine target on each detection during hold
            if tag_found:
                tag_n_off, tag_e_off = self.body_to_ned(self.tag_x, self.tag_y)
                new_target_x = self.drone_x + tag_n_off
                new_target_y = self.drone_y + tag_e_off
                alpha = 0.9
                self.target_x = alpha * self.target_x + (1.0 - alpha) * new_target_x
                self.target_y = alpha * self.target_y + (1.0 - alpha) * new_target_y
                # Closed-loop yaw also runs in HOLDING — without it, PX4 yaw
                # drifts during long hovers and the tag appears rotated in
                # the camera frame even though the drone thinks it's locked.
                new_yaw, _ = self.compute_target_yaw_from_tag()
                if new_yaw is not None:
                    self.target_yaw = new_yaw
            self.send_hold_position(self.target_x, self.target_y, self.target_z)
            self.get_logger().info(
                f'Hold tag {self.current_target_tag_id()}: {self.hover_duration - elapsed:.1f}s left',
                throttle_duration_sec=1.0)
            if elapsed >= self.hover_duration:
                # Advance to next leg
                self.sequence_index += 1
                if self.sequence_index >= len(self.sequence):
                    self.get_logger().info('Sequence complete. Handing off to PX4 AUTO.LAND.')
                    self.start_auto_landing()
                else:
                    next_tag = self.current_target_tag_id()
                    fwd, right = self.transit_direction_body()
                    self.get_logger().info(
                        f'Transit to tag {next_tag} (LED yellow). '
                        f'Body-frame direction: forward={fwd:.2f}, right={right:.2f}')
                    self.set_led_color('yellow')
                    self.reset_tag_filter()
                    self.target_x = None  # will re-lock once we re-center on new tag
                    self.target_y = None
                    self.centered_since = None
                    self.tag_loss_start = None
                    self.transit_start_time = time.time()
                    self.state = HopState.TRANSIT

        elif self.state == HopState.TRANSIT:
            # Body-frame TRANSIT: fly forward (or backward, etc.) at
            # transit_speed in the drone's own body frame. No NED math, no
            # tag_map NED targets. Arrival is detected by the next target
            # tag becoming visible in TF — at which point CENTERING takes
            # over its body-frame chase.
            self.pause_offboard_setpoints(True)
            elapsed = (time.time() - self.transit_start_time
                       if self.transit_start_time is not None else 0.0)
            # Only acquire the next tag after min_transit_duration has elapsed.
            # Without this gate, when adjacent tags fit in one camera FoV,
            # the script skips TRANSIT entirely and the body-frame velocity
            # phase is never visible. The minimum duration forces real
            # forward/backward motion before centering takes over.
            if tag_found and elapsed >= self.min_transit_duration:
                self.get_logger().info(
                    f'Tag {self.current_target_tag_id()} acquired during '
                    f'transit ({elapsed:.1f}s in). Switching to CENTERING.')
                self.set_led_color(self.detection_led_color)
                self.reset_tag_filter()
                self.state = HopState.CENTERING
                return
            if elapsed > self.transit_timeout:
                self.get_logger().error(
                    f'TRANSIT timed out after {elapsed:.1f}s — tag '
                    f'{self.current_target_tag_id()} never acquired. '
                    f'Handing off to PX4 AUTO.LAND.')
                self.start_auto_landing()
                return
            fwd, right = self.transit_direction_body()
            vx_body = fwd * self.transit_speed
            vy_body = right * self.transit_speed
            self.get_logger().info(
                f'TRANSIT to tag {self.current_target_tag_id()}: body vel '
                f'[fwd={vx_body:.2f}, right={vy_body:.2f}] m/s, '
                f'elapsed {elapsed:.1f}s',
                throttle_duration_sec=1.0)
            self.send_position_setpoint(vx_body, vy_body, 0.0,
                                        fixed_z=self.target_z)

        elif self.state == HopState.AUTO_LANDING:
            # Hands off — PX4 owns the drone in AUTO.LAND. We do not publish
            # setpoints; we just watch for vehicle_land_detected to confirm
            # touchdown. PX4 disarms itself on landing, so we don't send a
            # disarm command here. RC mode flip would exit AUTO.LAND too,
            # but we don't fight it — script falls through to LANDED via
            # land_detected once the drone is on the ground.
            self.get_logger().info(
                f'AUTO.LAND in progress: alt={self.current_altitude:.2f}m, '
                f'nav_state={self.nav_state}',
                throttle_duration_sec=1.0)
            if self.land_detected:
                self.get_logger().info('AUTO.LAND complete (PX4 land_detected). '
                                       'Drone disarmed by PX4.')
                self.set_led_color('green')
                self.pause_offboard_setpoints(False, log=True)
                self.state = HopState.LANDED

        elif self.state == HopState.LANDED:
            airborne = (not self.land_detected
                        and self.current_altitude > self.min_takeoff_altitude)
            if airborne:
                self.get_logger().info('Takeoff detected after land. Resetting to SEARCHING.')
                self.set_led_color('black')
                self.state = HopState.SEARCHING
                self.sequence_index = 0
                self.target_x = self.target_y = self.target_z = None
                self.centered_since = None
                self.tag_loss_start = None
                self.detection_time = None
                self.hold_start_time = None
                self.reset_tag_filter()
                self.was_in_offboard = False
                self.offboard_requested = False
                self.target_yaw = None
                self.commanded_yaw = None
            else:
                self.get_logger().info('Landed and disarmed.', throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = TagHop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
