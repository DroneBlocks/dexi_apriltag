#!/usr/bin/env python3
"""
Tag Map Shuttle Test

Takes off, then flies back and forth between two AprilTag positions indefinitely.
Used to test tag map odometry — the drone should maintain accurate position
as it transitions between tags.

Usage:
    ros2 run dexi_apriltag tag_map_shuttle_test.py

Default behavior (DEXIGridScene):
  1. Take off to 2.8m above tag 0 at NED origin
  2. Fly to tag 56 at NED (5, 5) — nearest grid tag
  3. Fly back to tag 0 at NED (0, 0)
  4. Repeat until Ctrl+C
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from dexi_interfaces.msg import OffboardNavCommand
from px4_msgs.msg import VehicleLocalPosition
import time
import math


class TagMapShuttleTest(Node):
    def __init__(self):
        super().__init__('tag_map_shuttle_test')

        self.declare_parameter('flight_altitude', 2.8)  # meters
        self.declare_parameter('target_north', 5.0)  # NED North of destination tag
        self.declare_parameter('target_east', 5.0)   # NED East of destination tag
        self.declare_parameter('position_threshold', 0.5)  # meters - close enough to target
        self.declare_parameter('wait_at_ends', 3.0)  # seconds to hover at each end

        self.flight_altitude = self.get_parameter('flight_altitude').value
        self.target_north = self.get_parameter('target_north').value
        self.target_east = self.get_parameter('target_east').value
        self.position_threshold = self.get_parameter('position_threshold').value
        self.wait_at_ends = self.get_parameter('wait_at_ends').value

        # QoS for PX4 / offboard manager
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_publisher = self.create_publisher(
            OffboardNavCommand, '/dexi/offboard_manager', qos_profile)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, qos_profile)

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.position_valid = False

        time.sleep(1)
        self.get_logger().info('Tag Map Shuttle Test initialized')
        self.get_logger().info(
            f'Altitude: {self.flight_altitude}m, '
            f'Target: NED({self.target_north}, {self.target_east})')

    def local_position_callback(self, msg):
        self.drone_x = msg.x
        self.drone_y = msg.y
        self.drone_z = msg.z
        self.position_valid = True

    def send_command(self, command, distance_or_degrees=0.0, wait_time=2.0,
                     north=0.0, east=0.0, down=0.0, yaw=0.0):
        msg = OffboardNavCommand()
        msg.command = command
        msg.distance_or_degrees = distance_or_degrees
        msg.north = float(north)
        msg.east = float(east)
        msg.down = float(down)
        msg.yaw = float(yaw)
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f'Command: {command} (N={north:.1f} E={east:.1f} D={down:.1f})')
        time.sleep(wait_time)

    def wait_for_position(self, target_north, target_east, timeout=30.0):
        """Wait until the drone is near the target position."""
        start = time.time()
        while time.time() - start < timeout:
            if self.position_valid:
                dn = self.drone_x - target_north
                de = self.drone_y - target_east
                dist = math.sqrt(dn * dn + de * de)
                if dist < self.position_threshold:
                    self.get_logger().info(
                        f'Reached target ({target_north:.1f}, {target_east:.1f}) '
                        f'- actual ({self.drone_x:.2f}, {self.drone_y:.2f}), err={dist:.2f}m')
                    return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn(
            f'Timeout reaching ({target_north:.1f}, {target_east:.1f}) '
            f'- actual ({self.drone_x:.2f}, {self.drone_y:.2f})')
        return False

    def execute(self):
        try:
            # Startup sequence
            self.get_logger().info('Starting offboard heartbeat...')
            self.send_command('start_offboard_heartbeat', wait_time=3.0)

            self.get_logger().info('Arming...')
            self.send_command('arm', wait_time=3.0)

            self.get_logger().info(f'Taking off to {self.flight_altitude}m...')
            self.send_command('offboard_takeoff',
                              distance_or_degrees=self.flight_altitude,
                              wait_time=8.0)

            # Home position (tag 0 at origin)
            home_north = self.drone_x
            home_east = self.drone_y
            target_down = -self.flight_altitude  # NED: negative = up

            self.get_logger().info(
                f'Home: NED({home_north:.2f}, {home_east:.2f}), '
                f'Target: NED({self.target_north}, {self.target_east})')

            lap = 0
            while rclpy.ok():
                lap += 1

                # Fly to destination tag
                self.get_logger().info(
                    f'--- Lap {lap}: Flying to tag at '
                    f'NED({self.target_north}, {self.target_east}) ---')
                self.send_command('goto_ned',
                                  north=self.target_north,
                                  east=self.target_east,
                                  down=target_down,
                                  yaw=0.0,
                                  wait_time=1.0)
                self.wait_for_position(self.target_north, self.target_east)

                self.get_logger().info(f'Holding at destination for {self.wait_at_ends}s...')
                time.sleep(self.wait_at_ends)

                # Fly back to home (tag 0)
                self.get_logger().info(
                    f'--- Lap {lap}: Flying back to home '
                    f'NED({home_north:.1f}, {home_east:.1f}) ---')
                self.send_command('goto_ned',
                                  north=home_north,
                                  east=home_east,
                                  down=target_down,
                                  yaw=0.0,
                                  wait_time=1.0)
                self.wait_for_position(home_north, home_east)

                self.get_logger().info(f'Holding at home for {self.wait_at_ends}s...')
                time.sleep(self.wait_at_ends)

                self.get_logger().info(f'Lap {lap} complete')

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted - landing...')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.send_command('land', wait_time=8.0)
            self.send_command('disarm', wait_time=2.0)
            self.send_command('stop_offboard_heartbeat', wait_time=1.0)
            self.get_logger().info('Shuttle test complete')


def main(args=None):
    rclpy.init(args=args)
    node = TagMapShuttleTest()
    try:
        node.execute()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
