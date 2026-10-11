#!/usr/bin/env python3
"""Hop back to the previous tag: take off over tag 1, center and hold, fly backward to tag 0, land on it.

Run on the drone or in the simulator with the DEXI workspace sourced:

    python3 ~/dexi_ws/src/dexi_apriltag/examples/python/hop_back_to_previous_tag.py

Start with the drone on the ground over tag 1 (where hop_to_next_tag.py lands),
nose still pointing away from tag 0: this hop flies backward.
The pilot can take over at any time by switching out of Offboard.

Every step is a service call (/dexi/flight/command and /dexi/tag_nav/command),
so each one waits until it is done and the mission stops if one fails.
    Use a service when you need to know it finished. Use a topic when you're
    streaming something many times a second, or just listening.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from dexi_interfaces.srv import FlightCommand, TagNavCommand


class DexiMission:
    """Flight steps go to the offboard manager, AprilTag steps go to tag_nav.

    Both are services, so each call returns when the step is done and raises
    RuntimeError if it failed.
    """

    def __init__(self):
        # Ctrl+C raises KeyboardInterrupt instead of shutting ROS down, so the
        # mission can still land.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node('dexi_mission')
        self.flight_client = self.node.create_client(FlightCommand, '/dexi/flight/command')
        self.tag_nav_client = self.node.create_client(TagNavCommand, '/dexi/tag_nav/command')
        self.flight_client.wait_for_service(timeout_sec=10.0)

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{request.command}: service {client.srv_name} is not available')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        print(f"{request.command}: {'ok' if result.success else 'FAILED'} {result.message}")
        if not result.success:
            raise RuntimeError(f'{request.command} failed: {result.message}')
        return result

    def flight(self, command, value=0.0, timeout=30.0):
        """One flight step on /dexi/flight/command: arm, offboard_takeoff, land, disarm, ..."""
        request = FlightCommand.Request()
        request.command = command
        request.value = float(value)
        request.timeout = float(timeout)
        return self._call(self.flight_client, request)

    def tag_nav(self, command, tag_id, timeout=0.0, hold_s=0.0, forward=0.0, right=0.0):
        """One AprilTag step on /dexi/tag_nav/command."""
        request = TagNavCommand.Request()
        request.command = command
        request.tag_id = int(tag_id)
        request.timeout = float(timeout)
        request.hold_s = float(hold_s)
        request.forward = float(forward)
        request.right = float(right)
        return self._call(self.tag_nav_client, request)

    def center_on_tag(self, tag_id, hold=1.0, timeout=25.0):
        """Center over the tag, then stay there for hold seconds, still correcting from the tag."""
        result = self.tag_nav('center_on_tag', tag_id, timeout=timeout)
        if hold > 0:
            result = self.tag_nav('hold_on_tag', tag_id, hold_s=hold)
        return result

    def fly_until_tag(self, direction, speed, tag_id, timeout=20.0):
        """Fly level forward/backward/left/right (relative to the nose) until the tag is seen."""
        forward, right = {'forward': (1, 0), 'backward': (-1, 0), 'right': (0, 1), 'left': (0, -1)}[direction]
        return self.tag_nav('fly_until_tag', tag_id, timeout=timeout, forward=forward * speed, right=right * speed)

    def land_on_tag(self, tag_id):
        self.center_on_tag(tag_id, hold=0)
        return self.flight('land', timeout=60.0)

    def flight_mode(self, wait_s=3.0):
        """Current PX4 mode name from /dexi/telemetry, or None if it is not publishing."""
        got = {}
        sub = self.node.create_subscription(String, '/dexi/telemetry', lambda m: got.setdefault('t', m.data), 10)
        t0 = time.time()
        while 't' not in got and time.time() - t0 < wait_s:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        return json.loads(got['t']).get('mode') if 't' in got else None

    def stop(self, reason):
        print(f'stopping: {reason}')
        # A step also fails when the pilot takes over by leaving Offboard. Landing then
        # would take the aircraft away from the pilot, so land only if the mission has it.
        mode = self.flight_mode()
        if mode == 'Offboard':
            try:
                self.flight('land', timeout=60.0)
            except RuntimeError as e:
                print(f'land failed: {e}')
        else:
            print(f'not landing: the pilot has the aircraft (mode {mode})')

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    mission = DexiMission()
    try:
        mission.flight('arm')
        mission.flight('offboard_takeoff', 1.5)
        mission.center_on_tag(1, hold=1)
        mission.fly_until_tag('backward', 0.25, 0)
        mission.land_on_tag(0)
    except RuntimeError as e:
        mission.stop(e)
    except KeyboardInterrupt:
        mission.stop('interrupted by user')
    finally:
        mission.shutdown()


if __name__ == '__main__':
    main()
