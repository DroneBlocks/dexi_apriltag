#!/usr/bin/env python3
"""Hop to the next tag: take off over tag 0, center, fly forward to tag 1, center, land.

Run on the drone or in the simulator with the DEXI workspace sourced:

    python3 ~/dexi_ws/src/dexi_apriltag/examples/python/hop_to_next_tag.py

Start with the drone on the ground over tag 0, nose pointing toward tag 1.
The pilot can take over at any time by switching out of Offboard.
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dexi_interfaces.srv import ExecuteBlocklyCommand


class DexiMission:
    """Flight commands go to the offboard manager; AprilTag commands go to tag_nav."""

    def __init__(self):
        rclpy.init()
        self.node = Node('dexi_mission')
        self.manager = self.node.create_client(ExecuteBlocklyCommand, '/dexi/execute_blockly_command')
        self.tag_nav_client = self.node.create_client(ExecuteBlocklyCommand, '/dexi/tag_nav/execute')
        self.manager.wait_for_service(timeout_sec=10.0)

    def _call(self, client, command, parameter=0.0, timeout=30.0, north=0.0, east=0.0, down=0.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{command}: service {client.srv_name} is not available')
        request = ExecuteBlocklyCommand.Request()
        request.command = command
        request.parameter = float(parameter)
        request.timeout = float(timeout)
        request.north, request.east, request.down = float(north), float(east), float(down)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        print(f"{command} {parameter if parameter else ''}: {'ok' if result.success else 'FAILED'} {result.message}")
        if not result.success:
            raise RuntimeError(f'{command} failed: {result.message}')
        return result

    def execute_command(self, command, parameter=0.0, timeout=30.0):
        return self._call(self.manager, command, parameter, timeout)

    def tag_nav(self, command, parameter=0.0, timeout=30.0, north=0.0, east=0.0, down=0.0):
        return self._call(self.tag_nav_client, command, parameter, timeout, north, east, down)

    def center_on_tag(self, tag_id, timeout=25.0):
        return self.tag_nav('center_on_tag', tag_id, timeout)

    def fly_until_tag(self, direction, speed, tag_id, timeout=20.0):
        """Fly level forward/backward/left/right (relative to the nose) until the tag is seen."""
        north, east = {'forward': (1, 0), 'backward': (-1, 0), 'right': (0, 1), 'left': (0, -1)}[direction]
        return self.tag_nav('fly_until_tag', tag_id, timeout, north=north * speed, east=east * speed)

    def land_on_tag(self, tag_id):
        self.center_on_tag(tag_id)
        return self.execute_command('land')

    def flight_mode(self, wait_s=3.0):
        """Current PX4 mode name from /dexi/telemetry, or None if it is not publishing."""
        got = {}
        sub = self.node.create_subscription(String, '/dexi/telemetry', lambda m: got.setdefault('t', m.data), 10)
        t0 = time.time()
        while 't' not in got and time.time() - t0 < wait_s:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        return json.loads(got['t']).get('mode') if 't' in got else None

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    mission = DexiMission()
    try:
        mission.execute_command('arm')
        mission.execute_command('offboard_takeoff', 1.5)
        mission.center_on_tag(0)
        mission.fly_until_tag('forward', 0.25, 1)
        mission.center_on_tag(1)
        mission.land_on_tag(1)
    except RuntimeError as e:
        print(f'stopping: {e}')
        # A step also fails when the pilot takes over by leaving Offboard. Landing then
        # would take the aircraft away from the pilot, so land only if the mission has it.
        mode = mission.flight_mode()
        if mode == 'Offboard':
            mission.execute_command('land')
        else:
            print(f'not landing: the pilot has the aircraft (mode {mode})')
    finally:
        mission.shutdown()


if __name__ == '__main__':
    main()
