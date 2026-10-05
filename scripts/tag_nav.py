#!/usr/bin/env python3
"""tag_nav: AprilTag navigation primitives behind one service.

The DroneBlocks GCS, Python and Node-RED all drive the aircraft through the
offboard manager. This node adds the tag-relative primitives the same way: it
reads the detector's TF, decides a body-frame velocity or a hold point, and
hands it to the manager as OffboardNavCommand set_velocity_body or hold_ned.
It never publishes a
trajectory setpoint of its own, so the manager stays the single owner of the
setpoint stream, the heartbeat and the hand-back to the pilot.

Service  /dexi/tag_nav/execute   dexi_interfaces/srv/ExecuteBlocklyCommand
    command=center_on_tag   parameter=<tag id, -1 = whichever tag is in view>  timeout=<s>
    command=wait_for_tag    parameter=<tag id, -1 = any>  timeout=<s>
    command=fly_until_tag   parameter=<tag id, -1 = any tag other than those in view at start>
                            north/east/down = body velocity (m/s, FRD)  timeout=<s>
        Flies at that velocity, holding height, and stops the moment the tag is seen
        twice in a row. The corridor hop: fly forward until the next tag, then center.
    command=wait_for_offboard  parameter=ignored  timeout=<s>
        Returns once the aircraft is armed, airborne, in OFFBOARD and the engage
        flag is set. This is the pilot hand-off: fly to a tag by hand, give the go
        (RC aux switch, Node-RED button, a block, a script), the mission continues.
Topic    /dexi/tag_nav/engage    std_msgs/Bool, the go signal, from anyone
Topic    /dexi/tag_nav/status    std_msgs/String, JSON, 5 Hz

The offset math (camera axes -> body axes -> body origin, mount offset applied
after the rotation, tag_scale on the raw range) and the chase law (speed
proportional inside centering_taper_dist, flat outside) come from
apriltag-corridor-mission-code. Loss handling follows it too: hold over where the tag was last seen (median of >= 3 samples, capped), and
give up after centering_loss_timeout.
"""

import json
import math
import threading
import time
from collections import deque

import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
try:
    from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks   # Iron and later
except ImportError:  # Humble
    from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks

from dexi_interfaces.msg import OffboardNavCommand
from dexi_interfaces.srv import ExecuteBlocklyCommand
from apriltag_msgs.msg import AprilTagDetectionArray
from px4_msgs.msg import ManualControlSetpoint, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Bool, String

PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
NAV_DETECTION_FRESH_S = 0.5
LOSS_HOLD_MAX = 0.60   # a genuine last sighting is <= ~0.6 m away; the tag leaves frame ~0.5 m behind the body
NAV_STATE_OFFBOARD = 14


class TagNav(Node):
    def __init__(self):
        super().__init__('tag_nav')
        p = self.declare_parameter
        # Mount. DEXI 5 v1 (ARK Pi6X + CM4): lens center 105 mm ahead
        # of the frame center, on the centerline, camera axes aligned with body
        # forward/right. A bench check with the tag centered in the image read
        # 3 cm, so the mount pitch is a true 90 deg. Per airframe: measure, never copy.
        p('camera_forward_offset', 0.105)
        p('camera_right_offset', 0.0)
        p('camera_yaw_deg', 0.0)
        p('tag_scale', 1.0)
        p('tag_family', 'tag36h11')
        # Chase law
        p('centering_speed', 0.20)        # m/s, flat outside the taper
        p('centering_taper_dist', 0.30)   # m, proportional inside
        # Floor on the chase speed outside the gate. Without one the aircraft can
        # park ~14 cm off on 0.06-0.09 m/s commands, which a flow EKF cannot tell
        # from noise; a 0.15 m/s floor limit-cycles ±0.2 m. Off by default.
        p('centering_min_speed', 0.0)     # m/s floor during the chase; 0 = off
        # Two-stage centering (CENTERING -> HOLDING): chase on velocity until
        # inside hold_enter, then hand the fine centering to PX4's position hold at
        # the tag's measured position, refined from every new detection.
        p('hold_enter', 0.25)             # m, switch to position hold inside this
        p('hold_exit', 0.40)              # m, drop back to the chase outside this
        p('hold_alpha', 0.8)              # EMA weight on the previous hold target (first fix only)
        # Hold refinement is integral, not "position + offset": on flow PX4 parks a
        # steady ~10 cm from the setpoint it is given, so the hold point is nudged by a fraction of what the camera still sees until the
        # camera reads zero. hold_nudge is that fraction per new detection.
        p('hold_nudge', 0.25)
        # Rate limit on the hold point: 0.012 m per detection at ~8 Hz is ~0.1 m/s, slow
        # enough for PX4 to follow without the point running ahead of the aircraft.
        p('hold_nudge_max', 0.012)
        # Anti-windup: only nudge once PX4 has settled on the point it was last given,
        # i.e. the aircraft is within hold_settled_m of the target and moving slower
        # than hold_settled_v. Nudging while it is still in transit piles corrections
        # on top of motion and the hold point runs away. The distance gate is loose
        # because on flow PX4 parks 10-18 cm from its setpoint. The speed gate is
        # tight because at 0.30 m/s the hold point chases an aircraft orbiting the
        # tag at 0.15-0.2 m/s (phase lag): integrate only when quasi-still.
        p('hold_settled_m', 0.35)
        p('hold_settled_v', 0.12)
        p('hold_accept_s', 8.0)           # s in hold, inside hold_enter, to accept without reaching the gate
        # Manager command used for the hold point. 'hold_ned' is a pure position
        # setpoint; 'goto_ned' declares arrival at 0.25 m and re-latches the hold at
        # the current position, which parks the aircraft ~20 cm off.
        p('hold_command', 'hold_ned')
        p('centering_gate', 0.10)         # m, centered when the offset is inside this
        p('centering_settle', 0.7)        # s inside the gate before reporting done
        p('filter_length', 5)
        p('tag_stale_s', 0.5)             # a TF that stops changing for this long is a lost tag
        p('tag_loss_grace', 3.0)          # s of loss before the settle timer resets
        p('centering_loss_timeout', 8.0)  # s of loss before the primitive fails
        # Altitude: the manager's velocity mode has no altitude hold of its own
        p('hold_altitude', True)
        p('alt_kz', 1.0)
        p('alt_max_rate', 0.3)
        # Safety
        p('require_airborne', True)       # rangefinder must be valid and above min_airborne_alt
        p('min_airborne_alt', 0.30)
        p('require_offboard', True)       # leaving OFFBOARD stands the primitive down
        p('dry_run', False)               # compute and publish status, send nothing to the manager
        # Hand-off: which RC aux channel raises engage (1..6, 0 = none) and the level
        # above which it counts as pressed. The press latches; release is fine.
        p('engage_aux_index', 0)
        p('engage_aux_threshold', 0.5)
        # With a dedicated PX4 offboard switch (RC_MAP_OFFB_SW), the flick into
        # OFFBOARD is itself the pilot's intent: count it as engage.
        p('engage_on_offboard', False)
        p('transit_min_s', 0.5)           # fly_until_tag: ignore sightings before this, so the tag being left cannot retrigger
        p('transit_confirm', 2)           # consecutive ticks the next tag must be seen
        p('loop_hz', 20.0)
        p('local_position_topic', '')  # '' = /fmu/out/vehicle_local_position (100 Hz); the launch file uses the manager's 20 Hz copy
        p('command_hz', 10.0)

        g = self.get_parameter
        self.cam_fwd_off = g('camera_forward_offset').value
        self.cam_right_off = g('camera_right_offset').value
        yaw = math.radians(g('camera_yaw_deg').value)
        self.cam_yaw_cos, self.cam_yaw_sin = math.cos(yaw), math.sin(yaw)
        self.tag_scale = g('tag_scale').value
        self.tag_family = g('tag_family').value
        self.centering_speed = g('centering_speed').value
        self.taper = g('centering_taper_dist').value
        self.min_speed = g('centering_min_speed').value
        self.hold_enter = g('hold_enter').value
        self.hold_exit = g('hold_exit').value
        self.hold_alpha = g('hold_alpha').value
        self.hold_nudge = g('hold_nudge').value
        self.hold_nudge_max = g('hold_nudge_max').value
        self.hold_settled_m = g('hold_settled_m').value
        self.hold_settled_v = g('hold_settled_v').value
        self.hold_accept_s = g('hold_accept_s').value
        self.hold_command = g('hold_command').value
        self.gate = g('centering_gate').value
        self.settle = g('centering_settle').value
        self.filter_length = int(g('filter_length').value)
        self.tag_stale_s = g('tag_stale_s').value
        self.loss_grace = g('tag_loss_grace').value
        self.loss_timeout = g('centering_loss_timeout').value
        self.hold_altitude = g('hold_altitude').value
        self.alt_kz = g('alt_kz').value
        self.alt_max_rate = g('alt_max_rate').value
        self.require_airborne = g('require_airborne').value
        self.min_airborne_alt = g('min_airborne_alt').value
        self.require_offboard = g('require_offboard').value
        self.dry_run = g('dry_run').value
        self.engage_aux_index = int(g('engage_aux_index').value)
        self.engage_aux_threshold = g('engage_aux_threshold').value
        self.engage_on_offboard = g('engage_on_offboard').value
        self.loop_hz = g('loop_hz').value
        self.local_position_topic = g('local_position_topic').value
        self.transit_min_s = g('transit_min_s').value
        self.transit_confirm = int(g('transit_confirm').value)
        self.command_period = 1.0 / g('command_hz').value

        self.cb = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Vehicle state
        self.lock = threading.RLock()
        self.pos = None            # (n, e, d)
        self.vel_xy = 0.0
        self.heading = 0.0
        self.xy_valid = False
        self.dist_bottom = None
        self.dist_bottom_valid = False
        self.pos_time = 0.0
        self.nav_state = -1
        self.armed = False
        self.engaged = False
        self.engage_changed = 0.0
        self.aux_was_high = False
        self.detected_ids = []
        self.detected_ms = 0.0

        fmu = self._fmu_prefix()
        # rclpy registers two QoS event handlers per subscription/publisher by default and
        # rebuilds the wait set on every wake; with a 100 Hz position stream that spin cost
        # 83% of a CM4 core while idle, none of it in this node's own code.
        NO_EVENTS = dict(event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False))
        NO_PUB_EVENTS = dict(event_callbacks=PublisherEventCallbacks(use_default_callbacks=False))
        self.create_subscription(VehicleLocalPosition, self.local_position_topic or f'{fmu}/vehicle_local_position',
                                 self._on_local_position, PX4_QOS, callback_group=self.cb, **NO_EVENTS)
        # PX4 1.16 boards publish vehicle_status under one of two names depending
        # on the message-versioning build; subscribe to both, one of them delivers.
        for name in (f'{fmu}/vehicle_status', f'{fmu}/vehicle_status_v1'):
            self.create_subscription(VehicleStatus, name, self._on_vehicle_status, PX4_QOS, callback_group=self.cb, **NO_EVENTS)
        # The manager's subscription is BEST_EFFORT + TRANSIENT_LOCAL; a default
        # (RELIABLE, VOLATILE) publisher is incompatible and silently ignored.
        cmd_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.cmd_pub = self.create_publisher(OffboardNavCommand, '/dexi/offboard_manager', cmd_qos, **NO_PUB_EVENTS)
        self.create_subscription(Bool, '/dexi/tag_nav/engage', self._on_engage, 10, callback_group=self.cb, **NO_EVENTS)
        if self.engage_aux_index >= 1:   # the RC stream runs at ~50 Hz; only pay for it when an aux channel is the engage switch
            self.create_subscription(ManualControlSetpoint, f'{fmu}/manual_control_setpoint',
                                     self._on_manual_control, PX4_QOS, callback_group=self.cb, **NO_EVENTS)
        self.create_subscription(AprilTagDetectionArray, '/apriltag_detections', self._on_detections, 10, callback_group=self.cb, **NO_EVENTS)
        self.status_pub = self.create_publisher(String, '/dexi/tag_nav/status', 10, **NO_PUB_EVENTS)
        # The service blocks for the length of a command, so it lives on its own node and its
        # own single-threaded executor (see main). That keeps the main executor single-threaded:
        # rclpy's MultiThreadedExecutor cost 83% of a CM4 core at idle just scheduling callbacks.
        # use_global_arguments=False: the launch remaps __node:=tag_nav for the whole process,
        # which would rename this node too and leave two nodes called 'tag_nav'.
        self.svc_node = Node('tag_nav_execute', use_global_arguments=False)
        self.svc_node.create_service(ExecuteBlocklyCommand, '/dexi/tag_nav/execute', self._on_execute)

        # Active job, driven by the control timer
        self.job = None
        self.create_timer(1.0 / self.loop_hz, self._control_tick, callback_group=self.cb)
        self.create_timer(0.2, self._publish_status, callback_group=self.cb)
        self.last_cmd_time = 0.0
        self.last_goto_time = 0.0
        self.last_cmd = (0.0, 0.0, 0.0)
        self.get_logger().info(
            f'tag_nav ready: mount fwd {self.cam_fwd_off:.3f} m right {self.cam_right_off:.3f} m '
            f'yaw {g("camera_yaw_deg").value:.0f} deg, chase {self.centering_speed} m/s taper {self.taper} m '
            f'gate {self.gate} m, dry_run={self.dry_run}, require_airborne={self.require_airborne}')

    # ----- PX4 plumbing -----
    def _fmu_prefix(self):
        return '/fmu/out'

    def _on_local_position(self, m):
        with self.lock:
            self.vel_xy = math.hypot(m.vx, m.vy) if hasattr(m, 'vx') else 0.0
            self.pos = (m.x, m.y, m.z)
            self.heading = m.heading
            self.xy_valid = bool(m.xy_valid)
            self.dist_bottom = m.dist_bottom
            self.dist_bottom_valid = bool(m.dist_bottom_valid)
            self.pos_time = time.time()

    def _on_vehicle_status(self, m):
        with self.lock:
            was_armed = self.armed
            entered_offboard = m.nav_state == NAV_STATE_OFFBOARD and self.nav_state != NAV_STATE_OFFBOARD
            self.nav_state = m.nav_state
            now_armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
            ekf_alt = -self.pos[2] if self.pos else 0.0
            airborne = (self.dist_bottom_valid and (self.dist_bottom or 0.0) > self.min_airborne_alt) or (now_armed and ekf_alt > self.min_airborne_alt)
            if entered_offboard and self.engage_on_offboard and not self.engaged and now_armed and airborne:
                self.engaged = True
                self.engage_changed = time.time()
                self.get_logger().info('engage SET (entered OFFBOARD)')
            self.armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
            if was_armed and not self.armed and self.engaged:
                self.engaged = False
                self.engage_changed = time.time()
                self.get_logger().info('engage CLEARED (disarmed)')

    def _on_engage(self, m):
        with self.lock:
            if bool(m.data) != self.engaged:
                self.engaged = bool(m.data)
                self.engage_changed = time.time()
                self.get_logger().info(f'engage {"SET" if self.engaged else "CLEARED"} (topic)')

    def _on_manual_control(self, m):
        if self.engage_aux_index < 1:
            return
        aux = [m.aux1, m.aux2, m.aux3, m.aux4, m.aux5, m.aux6][self.engage_aux_index - 1]
        high = aux > self.engage_aux_threshold
        if high and not self.aux_was_high:
            with self.lock:
                if not self.engaged:
                    self.engaged = True
                    self.engage_changed = time.time()
                    self.get_logger().info(f'engage SET (RC aux{self.engage_aux_index})')
        self.aux_was_high = high

    def _on_detections(self, m):
        with self.lock:
            self.detected_ids = [d.id for d in m.detections]
            self.detected_ms = time.time()

    def altitude(self):
        """Height above the floor, meters: rangefinder when valid, else the EKF."""
        with self.lock:
            if self.dist_bottom_valid and self.dist_bottom is not None:
                return float(self.dist_bottom)
            return -self.pos[2] if self.pos else 0.0

    def body_to_ned(self, fwd, right):
        c, s = math.cos(self.heading), math.sin(self.heading)
        return fwd * c - right * s, fwd * s + right * c

    def ned_to_body(self, dn, de):
        c, s = math.cos(self.heading), math.sin(self.heading)
        return dn * c + de * s, -dn * s + de * c

    # ----- Tag offset -----
    def tag_offset(self, tag_id, job):
        """Body-frame offset to the tag: (forward, right, down) in meters, or None."""
        frame = f'{self.tag_family}:{tag_id}'
        try:
            t = self.tf_buffer.lookup_transform('base_link', frame, rclpy.time.Time(),
                                                timeout=rclpy.duration.Duration(seconds=0.02))
        except Exception:
            return None
        tr = t.transform.translation
        cam_fwd = -tr.y * self.tag_scale
        cam_right = -tr.z * self.tag_scale
        fwd = cam_fwd * self.cam_yaw_cos - cam_right * self.cam_yaw_sin + self.cam_fwd_off
        right = cam_fwd * self.cam_yaw_sin + cam_right * self.cam_yaw_cos + self.cam_right_off
        down = tr.x * self.tag_scale
        # Stale-TF detection by the transform's own stamp: lookup keeps returning
        # the last pose after the tag leaves the frame, so a stamp that stops
        # advancing for tag_stale_s means the tag is gone. (Comparing values would
        # mistake a perfectly still aircraft on a bench for a lost tag.)
        now = time.time()
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if job['last_raw'] is not None and stamp <= job['last_raw'] + 1e-6:
            if now - job['last_change'] > self.tag_stale_s:
                return None
            job['new_sample'] = False
        else:
            job['last_change'] = now
            job['new_sample'] = True
        job['last_raw'] = stamp
        return fwd, right, down

    # ----- Commands to the manager -----
    def send_velocity(self, vx, vy, vz, force=False):
        now = time.time()
        if not force and now - self.last_cmd_time < self.command_period:
            return
        self.last_cmd_time = now
        self.last_cmd = (vx, vy, vz)
        if self.dry_run:
            return
        m = OffboardNavCommand()
        m.command = 'set_velocity_body'
        m.north, m.east, m.down, m.yaw = float(vx), float(vy), float(vz), 0.0
        self.cmd_pub.publish(m)

    def send_goto(self, n, e, d, yaw_deg, force=False):
        """Position hold at an NED point through the manager (hold_command)."""
        now = time.time()
        if not force and now - self.last_goto_time < 0.2:      # 5 Hz is plenty for a refined hold point
            return
        self.last_goto_time = now
        self.last_cmd = (0.0, 0.0, 0.0)
        if self.dry_run:
            return
        m = OffboardNavCommand()
        m.command = self.hold_command
        m.north, m.east, m.down, m.yaw = float(n), float(e), float(d), float(yaw_deg)
        self.cmd_pub.publish(m)

    def send_stop(self):
        self.last_cmd = (0.0, 0.0, 0.0)
        if self.dry_run:
            return
        m = OffboardNavCommand()
        m.command = 'stop_velocity'
        self.cmd_pub.publish(m)

    def alt_hold_vz(self, job, tag_down=None):
        """Hold height over the tag. While the tag is in view its measured range is
        the reference; when it is not, the EKF bridges from the last sighting.
        (The EKF height alone is not trustworthy here: with the rangefinder not
        fused it is baro, which can read 0.4 m low.)"""
        if not self.hold_altitude:
            return 0.0
        ekf = self.altitude()
        if tag_down is not None:
            if job['hold_alt'] is None:
                job['hold_alt'] = tag_down                 # capture on first sight
            job['ekf_at_sight'] = (ekf, tag_down)
            alt = tag_down
        elif job.get('ekf_at_sight'):
            ekf0, down0 = job['ekf_at_sight']
            alt = down0 + (ekf - ekf0)
        else:
            if job['hold_alt'] is None:
                job['hold_alt'] = ekf
            alt = ekf
        err = alt - job['hold_alt']                        # positive = too high
        return max(-self.alt_max_rate, min(self.alt_max_rate, self.alt_kz * err))   # NED: down positive

    # ----- Service -----
    def _on_execute(self, req, res):
        t0 = time.time()
        if self.job is not None:
            res.success, res.message = False, f'busy with {self.job["command"]}'
            return res
        cmd = req.command.strip()
        tag_id = int(round(req.parameter))
        timeout = req.timeout if req.timeout > 0 else 30.0
        if cmd not in ('center_on_tag', 'wait_for_tag', 'wait_for_offboard', 'fly_until_tag'):
            res.success, res.message = False, f'unknown command {cmd}'
            return res
        if cmd == 'wait_for_offboard':
            # Pilot hand-off: no preflight, this IS the wait for the aircraft to be ready.
            self.get_logger().info(f'waiting for hand-off (armed + airborne + OFFBOARD + engage), up to {timeout:.0f} s')
            t_end = t0 + timeout
            while time.time() < t_end:
                ok, why = self._preflight(quiet=True)
                with self.lock:
                    engaged = self.engaged
                    cancelled = (not self.engaged) and self.engage_changed > t0
                if cancelled:
                    res.success, res.message = False, 'hand-off cancelled'
                    res.execution_time = time.time() - t0
                    self.get_logger().info('hand-off cancelled')
                    return res
                if ok and engaged:
                    res.success, res.message = True, 'hand-off accepted'
                    res.execution_time = time.time() - t0
                    self.get_logger().info(f'hand-off accepted after {res.execution_time:.1f} s')
                    return res
                time.sleep(0.1)
            res.success, res.message = False, f'no hand-off within {timeout:.0f} s'
            res.execution_time = time.time() - t0
            return res
        ok, why = self._preflight()
        waited = 0.0
        while not ok and 'vehicle_local_position' in why and waited < 3.0:
            time.sleep(0.2); waited += 0.2
            ok, why = self._preflight()
        if not ok:
            res.success, res.message = False, why
            return res
        if tag_id < 0 and cmd == 'center_on_tag':
            # "any": take whichever tag the detector reports right now
            with self.lock:
                fresh = time.time() - self.detected_ms < 1.0
                ids = list(self.detected_ids)
            if not (fresh and ids):
                res.success, res.message = False, 'no tag in view to center on'
                return res
            tag_id = ids[0]
            self.get_logger().info(f'center_on_tag any -> tag {tag_id}')
        if cmd == 'fly_until_tag' and req.timeout <= 0:
            timeout = 20.0
        with self.lock:
            ids_at_start = set(self.detected_ids) if time.time() - self.detected_ms < 1.0 else set()
        job = {
            'command': cmd, 'tag_id': tag_id, 'timeout': timeout, 't0': t0,
            'vel': (float(req.north), float(req.east), float(req.down)), 'exclude': ids_at_start,
            'done': threading.Event(), 'success': False, 'message': '',
            'fx': deque(maxlen=self.filter_length), 'fy': deque(maxlen=self.filter_length),
            'last_raw': None, 'last_change': t0, 'new_sample': False,
            'last_seen': t0, 'loss_start': None, 'settled_since': None,
            'last_ned_hist': deque(maxlen=3), 'last_ned': None, 'hold_alt': None,
            'offset': None, 'hits': 0, 'log_t': 0.0,
            'mode': 'chase', 'hold_t0': None, 'target': None, 'target_d': None, 'yaw0': None,
        }
        if cmd == 'fly_until_tag':
            self.get_logger().info(f'fly_until_tag {tag_id if tag_id >= 0 else "any"}: body vel ({req.north:+.2f}, {req.east:+.2f}, {req.down:+.2f}) m/s, timeout {timeout:.0f} s, ignoring {sorted(ids_at_start)} at start')
        else:
            self.get_logger().info(f'{cmd} tag {tag_id}, timeout {timeout:.0f} s')
        self.job = job
        job['done'].wait(timeout + 1.0)
        if not job['done'].is_set():
            job['success'], job['message'] = False, 'timed out'
            self.send_stop()
        self.job = None
        res.success, res.message = job['success'], job['message']
        res.execution_time = time.time() - t0
        self.get_logger().info(f'{cmd} tag {tag_id}: {"ok" if res.success else "FAILED"} ({res.message}) in {res.execution_time:.1f} s')
        return res

    def _preflight(self, quiet=False):
        with self.lock:
            fresh = time.time() - self.pos_time < 1.0
            rng_ok = self.dist_bottom_valid and (self.dist_bottom or 0.0) > self.min_airborne_alt
            ekf_alt = -self.pos[2] if self.pos else 0.0
            ekf_ok = self.armed and ekf_alt > self.min_airborne_alt
            airborne = rng_ok or ekf_ok
            offboard = self.nav_state == NAV_STATE_OFFBOARD
            snap = (f'range={self.dist_bottom if self.dist_bottom is not None else "none"} valid={self.dist_bottom_valid} '
                    f'ekf_alt={ekf_alt:.2f} armed={self.armed} nav_state={self.nav_state}')
        if not fresh:
            return False, 'no vehicle_local_position from PX4'
        if self.require_airborne and not airborne:
            if not quiet: self.get_logger().warn(f'refused: not airborne ({snap})')
            return False, f'not airborne ({snap})'
        if self.require_offboard and not offboard and not self.dry_run:
            if not quiet: self.get_logger().warn(f'refused: not in OFFBOARD ({snap})')
            return False, f'not in OFFBOARD ({snap})'
        if not quiet: self.get_logger().info(f'gates ok ({snap})')
        return True, ''

    # ----- Control loop -----
    def _finish(self, job, success, message, keep_hold=False):
        # On success out of HOLD the manager is left holding the tag position, so
        # whatever block follows (a wait, a hop) starts from a real position hold
        # rather than a zero-velocity hold that drifts on flow.
        if not keep_hold:
            self.send_stop()
        job['success'], job['message'] = success, message
        job['done'].set()

    def _control_tick(self):
        job = self.job
        if job is None or job['done'].is_set():
            return
        now = time.time()
        if now - job['t0'] > job['timeout']:
            return self._finish(job, False, 'timed out')
        if self.require_offboard and not self.dry_run and self.nav_state != NAV_STATE_OFFBOARD:
            return self._finish(job, False, 'left OFFBOARD, standing down')
        with self.lock:
            cleared = (not self.engaged) and self.engage_changed > job['t0']
        if cleared:
            return self._finish(job, False, 'engage cleared, standing down')

        if job['command'] == 'fly_until_tag':
            vx, vy, vz = job['vel']
            elapsed = now - job['t0']
            with self.lock:
                fresh = time.time() - self.detected_ms < NAV_DETECTION_FRESH_S
                ids = list(self.detected_ids) if fresh else []
            if job['tag_id'] >= 0:
                seen = job['tag_id'] in ids
            else:
                seen = any(i not in job['exclude'] for i in ids)
            if elapsed < self.transit_min_s:
                seen = False
            job['hits'] = job['hits'] + 1 if seen else 0
            if job['hits'] >= self.transit_confirm:
                found = job['tag_id'] if job['tag_id'] >= 0 else next(i for i in ids if i not in job['exclude'])
                return self._finish(job, True, f'tag {found} seen after {elapsed:.1f} s')
            vertical = abs(vz) > 1e-3
            self.send_velocity(vx, vy, vz if vertical else self.alt_hold_vz(job))
            if now - job['log_t'] > 1.0:
                job['log_t'] = now
                self.get_logger().info(f'transit {elapsed:.1f} s: vel ({vx:+.2f}, {vy:+.2f}) alt {self.altitude():.2f}, tags in view {ids}')
            return

        off = self.tag_offset(job['tag_id'], job)
        job['offset'] = off

        if job['command'] == 'wait_for_tag':
            if job['tag_id'] < 0:
                with self.lock:
                    off = (0, 0, 0) if (time.time() - self.detected_ms < 0.5 and self.detected_ids) else None
            job['hits'] = job['hits'] + 1 if off is not None else 0
            if job['hits'] >= 2:
                return self._finish(job, True, f'tag {job["tag_id"]} in view')
            return

        # center_on_tag
        if off is None:
            if job['loss_start'] is None:
                job['loss_start'] = now
            lost_for = now - job['loss_start']
            if lost_for > self.loss_grace:
                job['settled_since'] = None
            if lost_for > self.loss_timeout:
                return self._finish(job, False, f'lost tag {job["tag_id"]} for {lost_for:.1f} s', keep_hold=(job['mode'] == 'hold'))
            if job['mode'] == 'hold' and job['target'] is not None:
                # The hold point is latched; PX4 keeps the aircraft there while we wait
                self.send_goto(job['target'][0], job['target'][1], job['target_d'], job['yaw0'])
                if now - job['log_t'] > 1.0:
                    job['log_t'] = now
                    self.get_logger().warn(f'tag {job["tag_id"]} lost {lost_for:.1f} s, holding the latched position')
                return
            # Chase-phase loss: creep back toward where it was last seen (>= 3 samples, capped)
            vx = vy = 0.0
            if job['last_ned'] is not None and len(job['last_ned_hist']) >= 3 and self.pos is not None:
                dn, de = job['last_ned'][0] - self.pos[0], job['last_ned'][1] - self.pos[1]
                dist = math.hypot(dn, de)
                if dist > LOSS_HOLD_MAX:
                    dn, de = dn * LOSS_HOLD_MAX / dist, de * LOSS_HOLD_MAX / dist
                    dist = LOSS_HOLD_MAX
                fwd, right = self.ned_to_body(dn, de)
                if dist > 0.02:
                    speed = min(1.0, dist / self.taper) * self.centering_speed
                    vx, vy = fwd / dist * speed, right / dist * speed
            self.send_velocity(vx, vy, self.alt_hold_vz(job))
            if now - job['log_t'] > 1.0:
                job['log_t'] = now
                self.get_logger().warn(f'tag {job["tag_id"]} lost {lost_for:.1f} s, creeping back at ({vx:.2f}, {vy:.2f})')
            return

        job['loss_start'] = None
        job['last_seen'] = now
        fwd, right, down = off
        raw_mag = math.hypot(fwd, right)
        if job['new_sample']:
            job['fx'].append(fwd)
            job['fy'].append(right)
            if self.pos is not None:
                tn, te = self.body_to_ned(fwd, right)
                job['last_ned_hist'].append((self.pos[0] + tn, self.pos[1] + te))
                xs = sorted(p[0] for p in job['last_ned_hist'])
                ys = sorted(p[1] for p in job['last_ned_hist'])
                job['last_ned'] = (xs[len(xs) // 2], ys[len(ys) // 2])

        # Stage switch with hysteresis
        if job['mode'] == 'chase' and raw_mag < self.hold_enter and self.pos is not None:
            job['mode'] = 'hold'
            job['hold_t0'] = now
            job['target'] = None
            job['yaw0'] = math.degrees(self.heading)
            self.get_logger().info(f'tag {job["tag_id"]} inside {self.hold_enter:.2f} m, handing fine centering to position hold')
        elif job['mode'] == 'hold' and raw_mag > self.hold_exit:
            job['mode'] = 'chase'
            job['target'] = None
            self.get_logger().warn(f'tag {job["tag_id"]} drifted to {raw_mag:.2f} m, back to the chase')

        # Settle / accept
        if raw_mag < self.gate:
            if job['settled_since'] is None:
                job['settled_since'] = now
            if now - job['settled_since'] >= self.settle:
                return self._finish(job, True, f'centered on tag {job["tag_id"]}: {raw_mag * 100:.0f} cm', keep_hold=(job['mode'] == 'hold'))
        else:
            job['settled_since'] = None
        if job['mode'] == 'hold' and now - job['hold_t0'] >= self.hold_accept_s and raw_mag < self.hold_enter:
            return self._finish(job, True, f'holding on tag {job["tag_id"]}: {raw_mag * 100:.0f} cm', keep_hold=True)

        if job['mode'] == 'hold':
            # Refine the hold point from every new detection
            if job['new_sample'] or job['target'] is None:
                tn, te = self.body_to_ned(fwd, right)
                new_n, new_e = self.pos[0] + tn, self.pos[1] + te
                if job['hold_alt'] is None:
                    job['hold_alt'] = down
                new_d = self.pos[2] + (down - job['hold_alt'])      # the EKF z at which the tag range equals the hold height
                if job['target'] is None:
                    job['target'], job['target_d'] = (new_n, new_e), new_d
                else:
                    # Nudge the hold point by part of the remaining camera offset (NED),
                    # capped per sample, but only while PX4 is settled on the previous
                    # point. That is what lets it converge when PX4 holds short of the
                    # setpoint without running away while it is still in transit.
                    at_target = math.hypot(self.pos[0] - job['target'][0], self.pos[1] - job['target'][1]) < self.hold_settled_m
                    settled = at_target and self.vel_xy < self.hold_settled_v
                    if settled and self.hold_nudge > 0:
                        k = self.hold_nudge
                        dn, de = k * tn, k * te
                        step = math.hypot(dn, de)
                        if step > self.hold_nudge_max:
                            dn, de = dn * self.hold_nudge_max / step, de * self.hold_nudge_max / step
                        job['target'] = (job['target'][0] + dn, job['target'][1] + de)
                    a = self.hold_alpha
                    job['target_d'] = a * job['target_d'] + (1 - a) * new_d
            self.send_goto(job['target'][0], job['target'][1], job['target_d'], job['yaw0'])
            if now - job['log_t'] > 0.5:
                job['log_t'] = now
                self.get_logger().info(
                    f'holding on tag {job["tag_id"]}: fwd {fwd:+.2f} right {right:+.2f} down {down:.2f} m '
                    f'-> target N {job["target"][0]:+.2f} E {job["target"][1]:+.2f} D {job["target_d"]:+.2f} '
                    f'pos N {self.pos[0]:+.2f} E {self.pos[1]:+.2f} v {self.vel_xy:.2f} hold {job["hold_alt"]:.2f}')
            return

        # Chase on the filtered offset: proportional inside the taper, flat outside.
        ex = sum(job['fx']) / len(job['fx']) if job['fx'] else fwd
        ey = sum(job['fy']) / len(job['fy']) if job['fy'] else right
        mag = math.hypot(ex, ey)
        if mag > 0.01:
            speed = min(1.0, mag / self.taper) * self.centering_speed
            if self.min_speed > 0:
                speed = max(speed, self.min_speed)
            vx, vy = ex / mag * speed, ey / mag * speed
        else:
            vx = vy = 0.0
        self.send_velocity(vx, vy, self.alt_hold_vz(job, down))
        if now - job['log_t'] > 0.5:
            job['log_t'] = now
            self.get_logger().info(
                f'centering on tag {job["tag_id"]}: fwd {fwd:+.2f} right {right:+.2f} down {down:.2f} m '
                f'-> vx {vx:+.2f} vy {vy:+.2f} vz {self.last_cmd[2]:+.2f} hold {job["hold_alt"] if job["hold_alt"] is not None else float("nan"):.2f} ekf {self.altitude():.2f}')

    def _publish_status(self):
        job = self.job
        with self.lock:
            st = {
                'state': 'idle' if job is None else job['command'],
                'tag': None if job is None else job['tag_id'],
                'visible': bool(job and job['offset'] is not None),
                'offset': None if not (job and job['offset']) else
                          {'forward': round(job['offset'][0], 3), 'right': round(job['offset'][1], 3), 'down': round(job['offset'][2], 3)},
                'cmd': {'vx': round(self.last_cmd[0], 3), 'vy': round(self.last_cmd[1], 3), 'vz': round(self.last_cmd[2], 3)},
                'alt': round(self.altitude(), 3) if self.pos else None,
                'range_valid': self.dist_bottom_valid,
                'nav_state': self.nav_state, 'armed': self.armed, 'engaged': self.engaged,
                'dry_run': self.dry_run,
            }
        self.status_pub.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = TagNav()
    # Two single-threaded executors: subscriptions + control loop on this thread, the blocking
    # service on a second thread. State shared between them is guarded by node.lock (RLock).
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex_svc = SingleThreadedExecutor()
    ex_svc.add_node(node.svc_node)
    threading.Thread(target=ex_svc.spin, name='tag_nav_execute', daemon=True).start()
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # On shutdown the context may already be gone; a stop that cannot be sent is not an error.
        try:
            if rclpy.ok():
                node.send_stop()
        except Exception:
            pass
        node.svc_node.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
