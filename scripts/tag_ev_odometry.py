#!/usr/bin/env python3
"""
AprilTag map -> PX4 external vision (EV) position for EKF2 fusion.

Each /apriltag_detections message (from apriltag_node on the downward camera) is turned
into one absolute pose fix in PX4's local NED frame:

  1. per tag in the map: PnP on the four corners (IPPE_SQUARE) -> tag pose in the camera
  2. camera -> body with the fixed downward mount (image top = forward, image right = right,
     lens camera_forward_offset ahead of and camera_down_offset below the body center)
  3. yaw from the tag's known bearing on the floor; roll and pitch from PX4's own attitude
     at the image time (the EKF's tilt is far better than one tag's)
  4. body position = tag position in the map - R(yaw, pitch, roll) * tag position in body
  5. several tags: inverse-variance mean

and published on /fmu/in/vehicle_visual_odometry (POSE_FRAME_NED, q with the tag yaw,
timestamp_sample = the image capture time). The map is in the SAME NED frame as PX4's GPS
origin (north/east meters from the EKF origin), so when GPS drops EKF2 resets onto the tag
frame without changing frames.

Variance grows with range and shrinks with the number of tags; PX4 uses
max(EKF2_EVP_NOISE^2, reported) when EKF2_EV_NOISE_MD = 0.

GPS-frame anchor (anchor_enable): the surveyed map and the GPS frame differ by the GPS error
(meters). While GNSS is fused PX4 hides that in its EV position bias estimator, then resets
the EKF onto the raw EV fix when GNSS stops: a jump of the full GPS error. With the anchor,
while GNSS is fused and an anchor tag (the X tag, and the apron/threshold tags) is seen with
the aircraft slow, this node
measures offset = EKF local xy - raw tag xy (median over anchor_window_s, then followed at up
to anchor_max_rate so GPS wander moves EV smoothly) and publishes every fix as raw + offset, so
EV agrees with the GPS frame and the reset at GNSS loss is only the drift of the stale-GPS
seconds. With anchor_apply_at_freeze (default) the offset is only measured while GNSS is fused
(raw fixes go out, so PX4's EV bias estimator keeps the EKF locally tag-stable as before) and
applied at the freeze; otherwise it is applied once measured and then follows the GPS. Either
way the switch bumps reset_counter, so PX4 re-seeds its bias estimator (to ~0) instead of
walking it over. The offset follows EKF xy resets while GNSS is fused and is frozen for good
once GNSS fusion stops or the GPS feed goes stale; it is never estimated indoors. No anchor by then = raw fixes, as without the anchor. Only xy: heading is
already the tag's (EV yaw is fused over the X), and EV height is fused against the baro height
reference through PX4's own EV height bias estimator. The price: indoor absolute (lat/lon)
position inherits the GPS error at the anchor; tag-relative flying does not care.

    ros2 run dexi_apriltag tag_ev_odometry.py --ros-args --params-file config/tag_ev_handover.yaml
"""

import bisect
import csv
import math
import os
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry, VehicleAttitude, VehicleLocalPosition, EstimatorStatusFlags, SensorGps


def quat_to_rot(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def rot_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_from_rot(R):
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def rot_from_euler(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rot_z(yaw) @ Ry @ Rx


def project(lat, lon, ref_lat, ref_lon):
    """PX4 MapProjection (azimuthal equidistant): lat/lon -> north, east meters from ref."""
    lat0, lon0, la, lo = map(math.radians, (ref_lat, ref_lon, lat, lon))
    cos_d = math.cos(lo - lon0)
    arg = max(-1.0, min(1.0, math.sin(lat0) * math.sin(la) + math.cos(lat0) * math.cos(la) * cos_d))
    c = math.acos(arg)
    k = 1.0 if abs(c) < 1e-12 else c / math.sin(c)
    r = 6371000.0
    return (k * (math.cos(lat0) * math.sin(la) - math.sin(lat0) * math.cos(la) * cos_d) * r,
            k * math.cos(la) * math.sin(lo - lon0) * r)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class TagEvOdometry(Node):
    def __init__(self):
        super().__init__('tag_ev_odometry')
        p = self.declare_parameter
        p('tag_ids', [0]); p('tag_north', [0.0]); p('tag_east', [0.0]); p('tag_down', [0.0])
        p('tag_top_bearing_deg', [0.0]); p('tag_size', 0.1524)
        # Per-id black-edge sizes, same order as tag_ids ([] = tag_size for every tag). Range gate
        # and sigmas scale with range / size: a 0.75 m tag at 5 m reads like a 6 in tag at 1 m.
        p('tag_sizes', [0.0])
        # The map's own origin (tag north/east/down are meters from it, down from its ground).
        # Tags are moved into PX4's local frame with the EKF origin (ref_lat/lon/alt), so a map
        # surveyed in a site frame lines up with GPS. NaN latitude = map already in EKF frame.
        p('map_origin_lat', float('nan')); p('map_origin_lon', float('nan')); p('map_origin_alt', float('nan'))
        # Downward camera mount on the body (FRD, meters).
        p('camera_forward_offset', 0.105); p('camera_right_offset', 0.0); p('camera_down_offset', 0.08)
        # Extra fixed yaw between the map's top_bearing and the printed top, if a map differs.
        p('tag_yaw_offset_deg', 0.0)
        p('max_range', 4.0)               # m, camera to tag
        p('min_decision_margin', 20.0)
        p('max_reproj_px', 2.0)
        p('sigma_xy_base', 0.03); p('sigma_xy_per_m', 0.03)
        p('sigma_z_base', 0.03); p('sigma_z_per_m', 0.02)
        p('sigma_yaw_deg_base', 2.0); p('sigma_yaw_deg_per_m', 1.0)
        p('publish_yaw', True)
        p('stamp_offset_ms', 0.0)         # added to the image stamp (negative = older)
        # GPS-frame anchor (see the module docstring)
        p('anchor_enable', False)
        p('anchor_tag_ids', [0])
        p('anchor_min_s', 2.0)            # s of steady anchor-tag samples before the first offset
        p('anchor_window_s', 5.0)         # s, rolling median window while GNSS is fused
        p('anchor_min_samples', 15)
        p('anchor_max_speed', 0.15)       # m/s EKF horizontal speed for a sample
        p('anchor_max_spread', 0.15)      # m, 90th percentile distance of samples from the median
        p('anchor_gps_stale_s', 1.0)      # no vehicle_gps_position for this long = GPS gone
        # After the first offset it follows the window median at most this fast (m/s), so GPS
        # wander moves EV smoothly instead of in steps; 0 = keep the first offset (no tracking).
        p('anchor_max_rate', 0.03)
        # true: keep publishing raw fixes while GNSS is fused (PX4's EV bias estimator keeps the
        # EKF locally tag-stable, as without the anchor) and only measure the offset; apply it,
        # with a reset_counter bump, at the freeze. false: apply it as soon as it is measured.
        p('anchor_apply_at_freeze', True)
        p('dry_run', False)
        p('csv_path', '/home/ubuntu/logs/tag_ev.csv')

        g = lambda n: self.get_parameter(n).value
        ids, tn, te, td, tb = g('tag_ids'), g('tag_north'), g('tag_east'), g('tag_down'), g('tag_top_bearing_deg')
        if not (len(ids) == len(tn) == len(te) == len(td) == len(tb)):
            raise RuntimeError('tag map arrays differ in length')
        self.tag_size = float(g('tag_size'))
        sizes = [float(v) for v in g('tag_sizes') if v > 0]
        if sizes and len(sizes) != len(ids):
            raise RuntimeError('tag_sizes must list one size per tag id (or be empty)')
        self.sizes = {int(i): (sizes[k] if sizes else self.tag_size) for k, i in enumerate(ids)}
        self.map = {}
        for i, n, e, d, b in zip(ids, tn, te, td, tb):
            # Tag frame (apriltag_ros PnP object points): x = print right, y = print down,
            # z = into the floor. With the print's top pointing bearing b on a level floor:
            top = math.radians(b + g('tag_yaw_offset_deg'))
            x_axis = np.array([math.cos(top + math.pi / 2), math.sin(top + math.pi / 2), 0.0])
            y_axis = np.array([math.cos(top + math.pi), math.sin(top + math.pi), 0.0])
            z_axis = np.cross(x_axis, y_axis)
            self.map[int(i)] = (np.array([n, e, d], dtype=float), np.column_stack([x_axis, y_axis, z_axis]))
        # apriltag corner order p0..p3 = tag (-1, 1), (1, 1), (1, -1), (-1, -1), which is also the
        # order SOLVEPNP_IPPE_SQUARE requires. Tag frame: x = print right, y = print down,
        # z = into the tag (so -y is the print's top).
        self.obj = {i: np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
                    for i, s in ((i, sz / 2) for i, sz in self.sizes.items())}
        # camera (x right, y down, z optical) -> body FRD for the downward mount
        self.R_bc = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        self.t_bc = np.array([g('camera_forward_offset'), g('camera_right_offset'), g('camera_down_offset')])
        self.K = None
        self.D = np.zeros(5)
        self.att = []   # (receive t_us wall, R_nb)
        self.px4_clock = None
        self.lpos = None
        self.dry_run = bool(g('dry_run'))
        self.publish_yaw = bool(g('publish_yaw'))
        self.stamp_offset_us = int(float(g('stamp_offset_ms')) * 1000)
        self.anchor_on = bool(g('anchor_enable'))
        self.apply_at_freeze = bool(g('anchor_apply_at_freeze'))
        self.anchor_ids = {int(i) for i in g('anchor_tag_ids')}
        self.anchor_state = 'waiting' if self.anchor_on else 'off'   # waiting -> set -> frozen | none
        self.anchor_off = None
        self.anchor_t = 0.0
        self.anchor_samples = []          # (wall s, dn, de)
        self.reset_counter = 0
        self.gnss = None; self.gnss_seen = False
        self.gps_t = None; self.gps_fix = 0
        self.lpos_hist = []               # (receive us, x, y, vx, vy, xy_valid)
        self.xy_rc = None

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos)
        self.dbg = self.create_publisher(String, '/dexi/tag_ev/debug', 10)
        self.create_subscription(AprilTagDetectionArray, '/apriltag_detections', self.on_det, 10)
        self.create_subscription(CameraInfo, '/cam0/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.on_att, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_lpos, qos_profile_sensor_data)
        if self.anchor_on:
            self.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags', self.on_flags, qos_profile_sensor_data)
            self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position', self.on_gps, qos_profile_sensor_data)

        path = g('csv_path')
        self.csv = None
        if path:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            self.csv_f = open(path, 'a', newline='')
            self.csv = csv.writer(self.csv_f)
            if self.csv_f.tell() == 0:
                self.csv.writerow(['wall_us', 'stamp_us', 'px4_stamp_us', 'latency_ms', 'n_tags', 'ids', 'north', 'east', 'down',
                                   'yaw_deg', 'sigma_xy', 'sigma_z', 'range_m', 'ekf_n', 'ekf_e', 'ekf_d', 'ekf_yaw_deg',
                                   'published', 'raw_north', 'raw_east', 'off_n', 'off_e', 'anchor', 'applied'])
        self.count = 0
        self._last_count = 0
        self.drops = {}
        self.create_timer(10.0, self.report)
        self.origin = (g('map_origin_lat'), g('map_origin_lon'), g('map_origin_alt'))
        self.map_shift = None if math.isfinite(self.origin[0]) else np.zeros(3)
        self.ref_stamp = None
        self.get_logger().info(f'tag EV: {len(self.map)} tags in map, dry_run={self.dry_run}, '
                               f'anchor apply {"at freeze" if self.apply_at_freeze else "when measured"}, '
                               f'publish_yaw={self.publish_yaw}, stamp_offset={self.stamp_offset_us / 1000:.0f} ms, '
                               f'anchor={"on, tags " + str(sorted(self.anchor_ids)) if self.anchor_on else "off"}')

    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=float).reshape(3, 3)
            if len(msg.d) >= 4:
                self.D = np.array(msg.d, dtype=float)
            self.get_logger().info(f'camera f={self.K[0, 0]:.1f} cx={self.K[0, 2]:.1f} cy={self.K[1, 2]:.1f}')

    def on_att(self, msg):
        # Keyed by OUR receive time, so the lookup never depends on PX4's clock. The newest
        # (PX4 stamp, receive time) pair also maps image stamps into PX4's clock (below).
        now_us = int(time.time() * 1e6)
        self.att.append((now_us, quat_to_rot(*[float(v) for v in msg.q])))
        self.px4_clock = (int(msg.timestamp_sample), now_us)
        if len(self.att) > 400:
            del self.att[:100]

    def on_lpos(self, msg):
        self.lpos = msg
        self.lpos_hist.append((int(time.time() * 1e6), msg.x, msg.y, msg.vx, msg.vy, bool(msg.xy_valid)))
        if len(self.lpos_hist) > 400:
            del self.lpos_hist[:100]
        if self.xy_rc is not None and msg.xy_reset_counter != self.xy_rc:
            # the EKF frame moved: old samples are in the old frame; a live offset moves with it
            self.anchor_samples.clear()
            if self.anchor_state == 'set':
                self.anchor_off = self.anchor_off + np.array([msg.delta_xy[0], msg.delta_xy[1]])
                self.get_logger().info(f'anchor: EKF xy reset {msg.delta_xy[0]:+.3f} {msg.delta_xy[1]:+.3f}, '
                                       f'offset now N {self.anchor_off[0]:+.3f} E {self.anchor_off[1]:+.3f}')
        self.xy_rc = msg.xy_reset_counter
        if not math.isfinite(self.origin[0]) or not msg.xy_global:
            return
        if self.ref_stamp == msg.ref_timestamp:
            return
        self.ref_stamp = msg.ref_timestamp
        self.anchor_samples.clear()   # the offset itself does not depend on the origin
        n, e = project(self.origin[0], self.origin[1], msg.ref_lat, msg.ref_lon)
        d = -(self.origin[2] - msg.ref_alt) if (math.isfinite(self.origin[2]) and msg.z_global) else 0.0
        self.map_shift = np.array([n, e, d])
        self.get_logger().info(f'EKF origin {msg.ref_lat:.7f}, {msg.ref_lon:.7f}, {msg.ref_alt:.2f}: '
                               f'map origin at N {n:.3f} E {e:.3f} D {d:.3f} in the EKF frame')

    def on_flags(self, msg):
        self.gnss = bool(msg.cs_gnss_pos)
        self.gnss_seen = self.gnss_seen or self.gnss
        self.anchor_freeze_check()

    def on_gps(self, msg):
        self.gps_t = time.time(); self.gps_fix = int(msg.fix_type)

    def gps_fresh(self):
        # never received (no such topic on this bridge): judge by the EKF flag alone
        return self.gps_t is None or (time.time() - self.gps_t < self.get_parameter('anchor_gps_stale_s').value
                                      and self.gps_fix >= 3)

    def anchor_freeze_check(self):
        if self.anchor_state not in ('waiting', 'set') or not self.gnss_seen:
            return
        gone = self.gnss is False or not self.gps_fresh()
        if self.anchor_state == 'set' and gone:
            self.anchor_state = 'frozen'
            if self.apply_at_freeze:
                self.reset_counter = (self.reset_counter + 1) % 256
            self.get_logger().info(f'anchor FROZEN at N {self.anchor_off[0]:+.3f} E {self.anchor_off[1]:+.3f} '
                                   f'(gnss_pos {int(bool(self.gnss))}, gps fresh {int(self.gps_fresh())})')
        elif self.anchor_state == 'waiting' and self.gnss is False:
            self.anchor_state = 'none'
            self.get_logger().warn('anchor: GNSS fusion stopped before an anchor was measured, raw tag fixes from here')

    def anchor_sample(self, fixes, stamp_us):
        """One EKF - raw sample from the anchor tags in this image, then the rolling offset."""
        g = lambda n: self.get_parameter(n).value
        self.anchor_freeze_check()
        if self.anchor_state not in ('waiting', 'set') or not self.gnss or not self.gps_fresh():
            return
        af = [f for f in fixes if f[0] in self.anchor_ids]
        lp = self.lpos_at(stamp_us)
        now = time.time()
        if af and lp is not None and lp[5] and math.hypot(lp[3], lp[4]) < g('anchor_max_speed'):
            w = np.array([1 / f[3] ** 2 for f in af])
            raw = sum(wi * f[1][:2] for wi, f in zip(w, af)) / w.sum()
            self.anchor_samples.append((now, lp[1] - raw[0], lp[2] - raw[1]))
        self.anchor_samples = [a for a in self.anchor_samples if now - a[0] <= g('anchor_window_s')]
        S = self.anchor_samples
        if len(S) < g('anchor_min_samples') or S[-1][0] - S[0][0] < g('anchor_min_s'):
            return
        d = np.array([[a[1], a[2]] for a in S])
        med = np.median(d, axis=0)
        spread = float(np.percentile(np.hypot(*(d - med).T), 90))
        if spread > g('anchor_max_spread'):
            return
        first = self.anchor_state == 'waiting'
        rate = float(g('anchor_max_rate'))
        if first or self.apply_at_freeze:
            self.anchor_off = med      # not published yet: no need to move it smoothly
        elif rate > 0:
            step = med - self.anchor_off
            lim = rate * max(0.0, now - self.anchor_t)
            n = float(np.hypot(*step))
            self.anchor_off = self.anchor_off + (step if n <= lim else step * (lim / n))
        self.anchor_t = now
        self.anchor_state = 'set'
        if first and not self.apply_at_freeze:
            # new EV frame: PX4 re-seeds its EV position bias (~0) instead of walking it over
            self.reset_counter = (self.reset_counter + 1) % 256
            self.get_logger().info(f'anchor SET N {med[0]:+.3f} E {med[1]:+.3f} ({len(S)} samples over '
                                   f'{S[-1][0] - S[0][0]:.1f} s, spread {spread:.3f} m), reset_counter {self.reset_counter}')

    def anchor_applied(self):
        return self.anchor_state == 'frozen' or (self.anchor_state == 'set' and not self.apply_at_freeze)

    def lpos_at(self, t_us):
        if not self.lpos_hist:
            return None
        ts = [a[0] for a in self.lpos_hist]
        i = bisect.bisect_left(ts, t_us)
        cand = [j for j in (i - 1, i) if 0 <= j < len(ts)]
        j = min(cand, key=lambda k: abs(ts[k] - t_us))
        return None if abs(ts[j] - t_us) > 100000 else self.lpos_hist[j]

    def attitude_at(self, t_us):
        if not self.att:
            return None
        ts = [a[0] for a in self.att]
        i = bisect.bisect_left(ts, t_us)
        cand = [j for j in (i - 1, i) if 0 <= j < len(ts)]
        j = min(cand, key=lambda k: abs(ts[k] - t_us))
        if abs(ts[j] - t_us) > 200000:
            return None
        return self.att[j][1]

    def drop(self, why):
        self.drops[why] = self.drops.get(why, 0) + 1

    def report(self):
        if self.drops or self.count != self._last_count:
            self.get_logger().info(f'fixes {self.count - self._last_count} in 10 s, drops {self.drops}, anchor {self.anchor_state}'
                                   + (f' N {self.anchor_off[0]:+.3f} E {self.anchor_off[1]:+.3f}' if self.anchor_off is not None else ''))
        self._last_count = self.count
        self.drops = {}

    def on_det(self, msg):
        if self.K is None or self.map_shift is None:
            self.drop('no camera_info or EKF origin')
            return
        g = lambda n: self.get_parameter(n).value
        stamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000 + self.stamp_offset_us
        R_nb_ekf = self.attitude_at(stamp_us)
        # timestamp_sample in PX4's clock as the agent reports it: anchored on the newest PX4
        # message instead of trusting wall clock == PX4 clock. They differ in SITL (lockstep runs
        # slower than real time, the uXRCE time sync then jumps by up to ~0.5 s) and on any
        # companion whose clock is not disciplined to the FC.
        px4_stamp_us = self.px4_clock[0] + (stamp_us - self.px4_clock[1]) if self.px4_clock else stamp_us
        if R_nb_ekf is None:
            self.drop('no attitude at image time')
            return
        roll, pitch, _ = euler_from_rot(R_nb_ekf)
        fixes = []
        for det in msg.detections:
            if det.id not in self.map:
                self.drop('id not in map'); continue
            if det.decision_margin < g('min_decision_margin'):
                self.drop('decision margin'); continue
            img = np.array([[c.x, c.y] for c in det.corners], dtype=np.float64)
            obj = self.obj[det.id]
            ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.D)
            reproj = float(np.max(np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)))
            rng = float(np.linalg.norm(tvec)) * self.tag_size / self.sizes[det.id]   # in default-size ranges
            if reproj > g('max_reproj_px'):
                self.drop('reprojection'); continue
            if rng > g('max_range'):
                self.drop('range'); continue
            R_ct, _ = cv2.Rodrigues(rvec)
            p_tag_b = self.t_bc + self.R_bc @ tvec.reshape(3)
            R_bt = self.R_bc @ R_ct
            tag_pos, R_nt = self.map[det.id]
            # yaw only from the tag; tilt from PX4
            _, _, yaw = euler_from_rot(R_nt @ R_bt.T)
            R_nb = rot_from_euler(roll, pitch, yaw)
            body = tag_pos + self.map_shift - R_nb @ p_tag_b
            sxy = g('sigma_xy_base') + g('sigma_xy_per_m') * rng
            sz = g('sigma_z_base') + g('sigma_z_per_m') * rng
            syaw = math.radians(g('sigma_yaw_deg_base') + g('sigma_yaw_deg_per_m') * rng)
            fixes.append((det.id, body, yaw, sxy, sz, syaw, rng))
        if not fixes:
            return
        wxy = np.array([1 / f[3] ** 2 for f in fixes])
        wz = np.array([1 / f[4] ** 2 for f in fixes])
        pos = np.zeros(3)
        pos[:2] = sum(w * f[1][:2] for w, f in zip(wxy, fixes)) / wxy.sum()
        pos[2] = sum(w * f[1][2] for w, f in zip(wz, fixes)) / wz.sum()
        wy = np.array([1 / f[5] ** 2 for f in fixes])
        y0 = fixes[0][2]
        yaw = wrap(y0 + sum(w * wrap(f[2] - y0) for w, f in zip(wy, fixes)) / wy.sum())
        var_xy, var_z, var_yaw = 1 / wxy.sum(), 1 / wz.sum(), 1 / wy.sum()
        raw_xy = pos[:2].copy()
        if self.anchor_on:
            self.anchor_sample(fixes, stamp_us)
            if self.anchor_applied():
                pos[:2] += self.anchor_off
        off = self.anchor_off if self.anchor_state in ('set', 'frozen') else np.zeros(2)

        out = VehicleOdometry()
        now_us = int(time.time() * 1e6)
        out.timestamp = now_us
        out.timestamp_sample = int(px4_stamp_us)
        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position = [float(pos[0]), float(pos[1]), float(pos[2])]
        nan = float('nan')
        if self.publish_yaw:
            q = rot_to_quat(rot_from_euler(roll, pitch, yaw))
            out.q = [float(v) for v in q]
            out.orientation_variance = [float(var_yaw)] * 3
        else:
            out.q = [nan, nan, nan, nan]
            out.orientation_variance = [nan, nan, nan]
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        out.velocity = [nan, nan, nan]
        out.angular_velocity = [nan, nan, nan]
        out.position_variance = [float(var_xy), float(var_xy), float(var_z)]
        out.velocity_variance = [nan, nan, nan]
        out.reset_counter = self.reset_counter
        out.quality = 100
        if not self.dry_run:
            self.pub.publish(out)
        self.count += 1
        lp = self.lpos
        ids = ' '.join(str(f[0]) for f in fixes)
        rng = min(f[6] for f in fixes)
        if self.csv:
            self.csv.writerow([now_us, stamp_us, int(px4_stamp_us), f'{(now_us - stamp_us) / 1000:.1f}', len(fixes), ids,
                               f'{pos[0]:.4f}', f'{pos[1]:.4f}', f'{pos[2]:.4f}', f'{math.degrees(yaw):.2f}',
                               f'{math.sqrt(var_xy):.4f}', f'{math.sqrt(var_z):.4f}', f'{rng:.3f}',
                               f'{lp.x:.4f}' if lp else '', f'{lp.y:.4f}' if lp else '', f'{lp.z:.4f}' if lp else '',
                               f'{math.degrees(lp.heading):.2f}' if lp else '', int(not self.dry_run),
                               f'{raw_xy[0]:.4f}', f'{raw_xy[1]:.4f}', f'{off[0]:.4f}', f'{off[1]:.4f}', self.anchor_state,
                               int(self.anchor_on and self.anchor_applied())])
            if self.count % 20 == 0:
                self.csv_f.flush()
        s = String()
        s.data = (f'{{"ids":[{ids.replace(" ", ",")}],"n":{pos[0]:.3f},"e":{pos[1]:.3f},"d":{pos[2]:.3f},'
                  f'"yaw_deg":{math.degrees(yaw):.2f},"sigma_xy":{math.sqrt(var_xy):.3f},"stamp_us":{stamp_us},'
                  f'"latency_ms":{(now_us - stamp_us) / 1000:.1f},"anchor":"{self.anchor_state}",'
                  f'"off_n":{off[0]:.3f},"off_e":{off[1]:.3f}}}')
        self.dbg.publish(s)
        if self.count % 50 == 1:
            self.get_logger().info(f'tags [{ids}] N {pos[0]:.2f} E {pos[1]:.2f} D {pos[2]:.2f} yaw {math.degrees(yaw):.1f} '
                                   f'sxy {math.sqrt(var_xy):.3f} lat {(now_us - stamp_us) / 1000:.0f} ms')


def main():
    rclpy.init()
    node = TagEvOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.csv:
            node.csv_f.flush()


if __name__ == '__main__':
    main()
