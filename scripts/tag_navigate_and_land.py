#!/usr/bin/env python3
"""
Tag Navigation + Precision Landing

Flies from current position to a target NED position using tag-based odometry,
then hands off to precision landing on a specific tag.

Usage:
    # Fly to NED(9,0) and land on tag 18
    ros2 run dexi_apriltag tag_navigate_and_land.py \
      --ros-args -p target_north:=9.0 -p target_east:=0.0 \
      -p flight_altitude:=2.0 -p landing_tag_id:=18
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from dexi_interfaces.msg import OffboardNavCommand
from px4_msgs.msg import VehicleLocalPosition
import time
import math
import subprocess
import signal
import os


class TagNavigateAndLand(Node):
    def __init__(self):
        super().__init__('tag_navigate_and_land')

        self.declare_parameter('flight_altitude', 2.0)
        self.declare_parameter('target_north', 9.0)
        self.declare_parameter('target_east', 0.0)
        self.declare_parameter('landing_tag_id', 18)
        self.declare_parameter('position_threshold', 0.25)
        self.declare_parameter('hover_before_landing', 3.0)  # seconds to hover at destination

        self.flight_altitude = self.get_parameter('flight_altitude').value
        self.target_north = self.get_parameter('target_north').value
        self.target_east = self.get_parameter('target_east').value
        self.landing_tag_id = self.get_parameter('landing_tag_id').value
        self.position_threshold = self.get_parameter('position_threshold').value
        self.hover_before_landing = self.get_parameter('hover_before_landing').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_pub = self.create_publisher(
            OffboardNavCommand, '/dexi/offboard_manager', qos)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.pos_cb, qos)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.pos_valid = False
        self.landing_process = None

        time.sleep(1)
        self.get_logger().info('Tag Navigate and Land initialized')
        self.get_logger().info(
            f'Target: NED({self.target_north}, {self.target_east}) at {self.flight_altitude}m')
        self.get_logger().info(f'Landing on tag {self.landing_tag_id}')

    def pos_cb(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.pos_valid = True

    def send_command(self, command, distance=0.0, wait=2.0, **kwargs):
        msg = OffboardNavCommand()
        msg.command = command
        msg.distance_or_degrees = distance
        msg.north = float(kwargs.get('north', 0.0))
        msg.east = float(kwargs.get('east', 0.0))
        msg.down = float(kwargs.get('down', 0.0))
        msg.yaw = float(kwargs.get('yaw', 0.0))
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Command: {command}')
        time.sleep(wait)

    def wait_for_position(self, target_n, target_e, timeout=60.0):
        start = time.time()
        while time.time() - start < timeout:
            if self.pos_valid:
                err = math.sqrt((self.x - target_n)**2 + (self.y - target_e)**2)
                if err < self.position_threshold:
                    self.get_logger().info(
                        f'Reached target! actual=({self.x:.2f},{self.y:.2f}) err={err:.2f}m')
                    return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn(
            f'Timeout reaching target. actual=({self.x:.2f},{self.y:.2f})')
        return False

    def start_precision_landing(self):
        """Launch precision landing node as a subprocess."""
        cmd = [
            'python3',
            '/home/dexi/dexi_ws/src/dexi_apriltag/scripts/precision_landing.py',
            '--ros-args',
            '-p', f'target_tag_id:={self.landing_tag_id}',
            '-p', 'detection_delay:=2.0',
            '-p', 'centering_threshold:=0.15',
            '-p', 'centering_speed:=0.10',
            '-p', 'descent_rate:=0.2',
        ]
        env = os.environ.copy()
        self.get_logger().info(f'Starting precision landing on tag {self.landing_tag_id}...')
        self.landing_process = subprocess.Popen(cmd, env=env)
        return self.landing_process

    def execute(self):
        try:
            # Wait for position data
            for i in range(30):
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.pos_valid:
                    break

            if not self.pos_valid:
                self.get_logger().error('No position data from PX4')
                return

            self.get_logger().info(
                f'Current position: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f})')

            # Phase 1: Takeoff
            self.get_logger().info('=== Phase 1: Takeoff ===')
            self.send_command('start_offboard_heartbeat', wait=3.0)
            self.send_command('arm', wait=3.0)
            self.send_command('offboard_takeoff',
                              distance=self.flight_altitude, wait=8.0)

            # Phase 2: Navigate to destination
            self.get_logger().info(
                f'=== Phase 2: Flying to NED({self.target_north}, {self.target_east}) ===')
            self.send_command('goto_ned',
                              north=self.target_north,
                              east=self.target_east,
                              down=-self.flight_altitude)
            self.wait_for_position(self.target_north, self.target_east)

            # Hover at destination
            self.get_logger().info(
                f'Hovering at destination for {self.hover_before_landing}s...')
            time.sleep(self.hover_before_landing)

            # Phase 3: Precision landing
            self.get_logger().info(
                f'=== Phase 3: Precision landing on tag {self.landing_tag_id} ===')
            proc = self.start_precision_landing()

            # Wait for landing to complete
            while proc.poll() is None:
                rclpy.spin_once(self, timeout_sec=0.5)
                # Check altitude — if very low, landing is done
                if self.pos_valid and -self.z < 0.2:
                    self.get_logger().info('Drone on ground — landing complete')
                    time.sleep(2)
                    break

            self.get_logger().info('=== Mission complete! ===')

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted')
        finally:
            # Clean up landing process
            if self.landing_process and self.landing_process.poll() is None:
                self.landing_process.send_signal(signal.SIGINT)
                self.landing_process.wait(timeout=5)

            self.send_command('stop_offboard_heartbeat', wait=1.0)
            self.get_logger().info('Done')


def main(args=None):
    rclpy.init(args=args)
    node = TagNavigateAndLand()
    try:
        node.execute()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
