#!/usr/bin/env python3
"""Tag sprint: take off on a start tag, fly down a line of tags and back, land on the start tag.

    start tag  ---- fast ---->  checkpoint tag  -- slow -->  finish tag
       ^                                                         |
       +-------------------------- fast -------------------------+   (--one-way skips this leg)

Run on the drone with the DEXI workspace sourced, as root (the DEXI nodes run as root):

    sudo -E python3 -u tag_sprint.py                       # tags 1 -> 4 -> 5 and back at 1.0 m/s
    sudo -E python3 -u tag_sprint.py --start 0 --checkpoint 3 --finish 6 \\
        --checkpoint-m 4.0 --finish-m 6.0 --speed 1.0

Then, in the field (full steps in docs/TAG_SPRINT.md):
  1. Put the drone on the ground over the start tag, nose toward the finish tag.
  2. Mode switch to Position, arm.
  3. Flip the Offboard switch. The drone takes off, flies the sprint and lands.
Abort at any time: flip the Offboard switch back. Sticks do not override Offboard.

Each step is a service call to the offboard manager (/dexi/flight/command) or tag_nav
(/dexi/tag_nav/command), so a step that fails stops the sprint and the drone holds where it is.

The legs:
  takeoff         offboard_takeoff to --height
  center          center_on_tag on the start tag (aligns heading to the tag)
  fast leg        fly_until_tag checkpoint at --speed; tag_nav brakes when it sees the tag
  slow leg        fly_until_tag finish at --slow (approach speed, keeps the stop on the tag)
  center          center_on_tag on the finish tag
  back            fly_until_tag start at -(--speed), passing the checkpoint; center on the start tag
  land            PX4 Land, with a forced disarm if the land detector never confirms touchdown

Every fly_until_tag has a timeout of 1.3 x the leg + 0.7 m of travel, so a missed tag stops the
drone a little past where the tag should be instead of flying on.
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleLandDetected, VehicleCommand
from dexi_interfaces.srv import FlightCommand, TagNavCommand

ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
ap.add_argument('--start', type=int, default=1, help='tag the drone takes off from and lands on')
ap.add_argument('--checkpoint', type=int, default=4, help='tag where the fast leg ends')
ap.add_argument('--finish', type=int, default=5, help='far tag')
ap.add_argument('--checkpoint-m', type=float, default=4.93, help='measured distance start -> checkpoint, m')
ap.add_argument('--finish-m', type=float, default=6.41, help='measured distance start -> finish, m')
ap.add_argument('--speed', type=float, default=1.0, help='fast leg speed, m/s (1.0 flown on DEXI-3 and DEXI-5)')
ap.add_argument('--slow', type=float, default=0.6, help='checkpoint -> finish approach speed, m/s')
ap.add_argument('--height', type=float, default=1.5, help='takeoff height, m')
ap.add_argument('--one-way', action='store_true', help='land on the finish tag instead of flying back')
ap.add_argument('--exposure-us', type=int, default=4000,
                help='fixed down-camera exposure in microseconds (0 = leave the camera alone). '
                     'Auto-exposure blurs tags above ~0.75 m/s.')
ap.add_argument('--gain', type=float, default=10.6, help='analogue gain used with --exposure-us')
args = ap.parse_args()

OFFBOARD, LAND, DESCEND = 14, 18, 12
GROUND_M, GROUND_S, LAND_MIN_S = 0.20, 6.0, 3.0   # forced-disarm backstop

rclpy.init()
node = Node('tag_sprint')
s = {'nav': -1, 'armed': False, 'landed': True, 'rng': 0.0, 'tn': {}}
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


def on_status(m):
    s['nav'] = m.nav_state
    s['armed'] = m.arming_state == 2


for topic in ('/fmu/out/vehicle_status', '/fmu/out/vehicle_status_v1'):
    node.create_subscription(VehicleStatus, topic, on_status, q)
node.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_20hz',
                         lambda m: s.__setitem__('rng', m.dist_bottom), q)
node.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected',
                         lambda m: s.__setitem__('landed', m.landed), q)
node.create_subscription(String, '/dexi/tag_nav/status', lambda m: s.__setitem__('tn', json.loads(m.data)), 10)
engage = node.create_publisher(Bool, '/dexi/tag_nav/engage', 10)
cmd_pub = node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', q)
flight = node.create_client(FlightCommand, '/dexi/flight/command')
tagnav = node.create_client(TagNavCommand, '/dexi/tag_nav/command')
for c in (flight, tagnav):
    while not c.wait_for_service(timeout_sec=5.0):
        print(f'waiting for {c.srv_name} (is tag_nav enabled in bringup?)', flush=True)


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def call(client, req):
    f = client.call_async(req)
    rclpy.spin_until_future_complete(node, f)
    r = f.result()
    log(f'{req.command}: {"ok" if r.success else "FAILED"} {r.message}')
    return r.success


def fc(cmd, value=0.0, timeout=0.0):
    r = FlightCommand.Request()
    r.command, r.value, r.timeout = cmd, float(value), float(timeout)
    return call(flight, r)


def tn(cmd, tag, timeout=0.0, fwd=0.0):
    r = TagNavCommand.Request()
    r.command, r.tag_id, r.timeout, r.forward = cmd, int(tag), float(timeout), float(fwd)
    return call(tagnav, r)


def leg_timeout(dist_m, v):
    return (1.3 * abs(dist_m) + 0.7) / abs(v) + 0.5


def offset_cm():
    o = s['tn'].get('offset') or {}
    return (o.get('forward', 0) ** 2 + o.get('right', 0) ** 2) ** 0.5 * 100 if o else float('nan')


def set_exposure():
    """Fixed short exposure on /cam0 (camera_ros). Reverts to auto when the camera restarts."""
    if args.exposure_us <= 0:
        return
    cli = node.create_client(SetParameters, '/cam0/set_parameters')
    if not cli.wait_for_service(timeout_sec=5.0):
        log('camera parameter service not found: exposure left as is')
        return
    vals = [('ExposureTimeMode', 1), ('ExposureTime', args.exposure_us), ('AnalogueGainMode', 1), ('AnalogueGain', args.gain)]
    for name, v in vals:   # one at a time: camera_ros needs the mode set before the value
        p = Parameter(name=name)
        if isinstance(v, float):
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
        else:
            p.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
        f = cli.call_async(SetParameters.Request(parameters=[p]))
        rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
    log(f'camera: exposure {args.exposure_us} us, gain {args.gain}')


def force_disarm():
    vc = VehicleCommand()
    vc.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
    vc.param1, vc.param2 = 0.0, 21196.0   # disarm, forced
    vc.target_system = vc.target_component = vc.source_system = vc.source_component = 1
    vc.from_external = True
    for _ in range(3):
        vc.timestamp = node.get_clock().now().nanoseconds // 1000
        cmd_pub.publish(vc)
        spin(0.1)


def land():
    """PX4 Land. If the land detector never confirms touchdown (height under GROUND_M for
    GROUND_S, at least LAND_MIN_S after the land started), disarm with a forced disarm."""
    r = FlightCommand.Request()
    r.command, r.value, r.timeout = 'land', 0.0, 30.0
    f = flight.call_async(r)
    t0, low_since, forced = time.time(), None, False
    while not f.done():
        rclpy.spin_once(node, timeout_sec=0.05)
        on_ground = s['armed'] and s['nav'] in (LAND, DESCEND) and s['rng'] < GROUND_M
        if not forced and on_ground and time.time() - t0 >= LAND_MIN_S:
            low_since = low_since or time.time()
            if time.time() - low_since >= GROUND_S:
                log(f"on the ground {GROUND_S:.0f} s and still armed: forced disarm")
                force_disarm()
                forced = True
        else:
            low_since = None
    ok = f.result().success
    log(f'land: {"ok" if ok else "FAILED"}' + (' (forced disarm)' if forced else ''))
    return ok


def hold_here(why):
    log(f'{why}: holding, take over with the Offboard switch')
    return False


def sprint():
    v, a = args.speed, args
    t0 = time.time()
    if not fc('offboard_takeoff', value=a.height, timeout=25.0) or s['nav'] != OFFBOARD:
        return hold_here('takeoff did not complete')
    log(f"takeoff: range {s['rng']:.2f} m at {time.time() - t0:.1f} s")
    engage.publish(Bool(data=True))
    spin(0.3)
    if not tn('center_on_tag', a.start, timeout=10.0):
        return hold_here(f'could not center on tag {a.start}')
    if not tn('fly_until_tag', a.checkpoint, timeout=leg_timeout(a.checkpoint_m, v), fwd=v):
        return hold_here(f'tag {a.checkpoint} not found')
    log(f'checkpoint tag {a.checkpoint} at {time.time() - t0:.1f} s')
    if not tn('fly_until_tag', a.finish, timeout=leg_timeout(a.finish_m - a.checkpoint_m, a.slow), fwd=a.slow):
        return hold_here(f'tag {a.finish} not found')
    if not tn('center_on_tag', a.finish, timeout=15.0):
        return hold_here(f'could not center on tag {a.finish}')
    end_tag = a.finish
    if not a.one_way:
        log(f'tag {a.finish} at {time.time() - t0:.1f} s: back to tag {a.start}')
        if not tn('fly_until_tag', a.start, timeout=leg_timeout(a.finish_m, v), fwd=-v):
            return hold_here(f'tag {a.start} not found')
        if not tn('center_on_tag', a.start, timeout=15.0):
            return hold_here(f'could not center on tag {a.start}')
        end_tag = a.start
    log(f"landing on tag {end_tag} at {time.time() - t0:.1f} s, offset {offset_cm():.1f} cm")
    ok = land()
    log(f"SPRINT {'ok' if ok else 'FAILED'} at {v:.2f} m/s: {time.time() - t0:.1f} s from the switch to disarmed")
    return ok


set_exposure()
fc('start_setpoint_stream')   # heartbeat without commanding Offboard, so PX4 accepts the switch flip
log(f'READY: on the ground over tag {args.start}, nose toward tag {args.finish}. '
    f'Position mode, arm, then flip the Offboard switch.')
prev = -1
while rclpy.ok():
    spin(0.05)
    flipped = s['nav'] == OFFBOARD and prev not in (OFFBOARD, -1)
    prev = s['nav']
    if not (flipped and s['armed'] and s['landed']):
        continue
    sprint()
    while s['armed']:               # a sprint that stopped early: wait for the pilot to land
        spin(0.1)
    fc('start_setpoint_stream')     # the land stopped the heartbeat
    prev = s['nav']
    log('READY for the next sprint: Offboard switch back, re-arm, flip it again')
