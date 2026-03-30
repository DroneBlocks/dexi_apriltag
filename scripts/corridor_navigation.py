#!/usr/bin/env python3
"""
Corridor Navigation using AprilTag odometry.

Flies along a corridor of AprilTags by stepping through centerline waypoints.
The AprilTag odometry node runs separately, feeding position corrections to
the PX4 EKF2. This script just sends goto_ned commands along the centerline.

If tags are lost for too long, the drone stops forward progress, descends to
widen the camera FOV, and retreats toward the last known good position.

Usage:
    ros2 run dexi_apriltag corridor_navigation.py

    # Custom parameters:
    ros2 run dexi_apriltag corridor_navigation.py --ros-args \
        -p flight_altitude:=1.5 \
        -p waypoint_spacing:=1.0 \
        -p corridor_length:=9.0 \
        -p corridor_east:=0.0

Parameters:
    flight_altitude: Cruising altitude in meters (default: 1.5)
    waypoint_spacing: Distance between centerline waypoints in meters (default: 1.0)
    corridor_length: Total corridor length in NED North meters (default: 9.0)
    corridor_east: East coordinate of corridor centerline (default: 0.0)
    position_threshold: How close to a waypoint before advancing (default: 0.3)
    forward_speed: Not used directly — PX4 offboard handles velocity
    tag_lost_timeout: Seconds without tags before recovery (default: 3.0)
    descend_step: Meters to descend during recovery (default: 0.3)
    retreat_step: Meters to retreat toward last good position (default: 0.5)
    min_altitude: Minimum altitude during recovery descent (default: 0.8)
    landing_tag_id: Tag ID to precision land on at destination (-1 = skip landing)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from dexi_interfaces.msg import OffboardNavCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
import time
import math


class CorridorNavigation(Node):
    def __init__(self):
        super().__init__('corridor_navigation')

        # Corridor parameters
        self.declare_parameter('flight_altitude', 1.5)
        self.declare_parameter('waypoint_spacing', 1.0)
        self.declare_parameter('corridor_length', 9.0)
        self.declare_parameter('corridor_east', 0.0)
        self.declare_parameter('position_threshold', 0.3)

        # Recovery parameters
        self.declare_parameter('tag_lost_timeout', 3.0)
        self.declare_parameter('descend_step', 0.3)
        self.declare_parameter('retreat_step', 0.5)
        self.declare_parameter('min_altitude', 0.8)

        # Landing
        self.declare_parameter('landing_tag_id', -1)

        self.flight_altitude = self.get_parameter('flight_altitude').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.corridor_length = self.get_parameter('corridor_length').value
        self.corridor_east = self.get_parameter('corridor_east').value
        self.position_threshold = self.get_parameter('position_threshold').value
        self.tag_lost_timeout = self.get_parameter('tag_lost_timeout').value
        self.descend_step = self.get_parameter('descend_step').value
        self.retreat_step = self.get_parameter('retreat_step').value
        self.min_altitude = self.get_parameter('min_altitude').value
        self.landing_tag_id = self.get_parameter('landing_tag_id').value

        # Build centerline waypoints
        self.waypoints = []
        north = 0.0
        while north <= self.corridor_length:
            self.waypoints.append((north, self.corridor_east))
            north += self.waypoint_spacing

        # QoS for PX4
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Offboard command publisher
        self.cmd_pub = self.create_publisher(
            OffboardNavCommand, '/dexi/offboard_manager', px4_qos)

        # Position subscriber
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.position_callback, px4_qos)

        # Visual odometry subscriber — track when tags are being fused
        self.last_odom_time = None
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
            self.odom_callback, px4_qos)

        # State
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.position_valid = False
        self.last_good_north = 0.0
        self.last_good_east = self.corridor_east
        self.current_altitude = self.flight_altitude

        time.sleep(1)
        self.get_logger().info('Corridor Navigation initialized')
        self.get_logger().info(f'Corridor: {self.corridor_length}m, {len(self.waypoints)} waypoints')
        self.get_logger().info(f'Altitude: {self.flight_altitude}m, centerline East: {self.corridor_east}')
        for i, (n, e) in enumerate(self.waypoints):
            self.get_logger().info(f'  WP{i}: NED({n:.1f}, {e:.1f})')

    def position_callback(self, msg):
        self.drone_x = msg.x
        self.drone_y = msg.y
        self.drone_z = msg.z
        self.position_valid = True

    def odom_callback(self, msg):
        """Track when visual odometry messages are being published."""
        self.last_odom_time = time.time()

    def tags_visible(self):
        """Check if tag odometry has published recently."""
        if self.last_odom_time is None:
            return False
        return (time.time() - self.last_odom_time) < 1.0

    def send_command(self, command, distance=0.0, wait=2.0, **kwargs):
        msg = OffboardNavCommand()
        msg.command = command
        msg.distance_or_degrees = distance
        msg.north = float(kwargs.get('north', 0.0))
        msg.east = float(kwargs.get('east', 0.0))
        msg.down = float(kwargs.get('down', 0.0))
        msg.yaw = float(kwargs.get('yaw', 0.0))
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'CMD: {command}')
        time.sleep(wait)

    def distance_to(self, target_north, target_east):
        """Horizontal distance from drone to target."""
        dn = self.drone_x - target_north
        de = self.drone_y - target_east
        return math.sqrt(dn * dn + de * de)

    def goto_waypoint(self, north, east, timeout=30.0):
        """Fly to a waypoint with tag-loss recovery. Returns True if reached."""
        down = -self.current_altitude
        self.send_command('goto_ned', north=north, east=east, down=down, yaw=0.0, wait=0.5)

        start = time.time()
        tag_lost_since = None

        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if not self.position_valid:
                continue

            # Check if we've arrived
            dist = self.distance_to(north, east)
            if dist < self.position_threshold:
                self.get_logger().info(
                    f'Reached WP NED({north:.1f}, {east:.1f}) — err={dist:.2f}m')
                self.last_good_north = self.drone_x
                self.last_good_east = self.drone_y
                return True

            # Track tag visibility
            if self.tags_visible():
                tag_lost_since = None
                self.last_good_north = self.drone_x
                self.last_good_east = self.drone_y
            elif tag_lost_since is None:
                tag_lost_since = time.time()

            # Recovery: tags lost for too long
            if tag_lost_since and (time.time() - tag_lost_since) > self.tag_lost_timeout:
                self.get_logger().warn('Tags lost — starting recovery')

                # Step 1: Stop and descend to widen FOV
                if self.current_altitude > self.min_altitude:
                    new_alt = max(self.current_altitude - self.descend_step, self.min_altitude)
                    self.get_logger().info(f'Descending: {self.current_altitude:.1f}m → {new_alt:.1f}m')
                    self.current_altitude = new_alt
                    self.send_command('goto_ned',
                                     north=self.drone_x, east=self.drone_y,
                                     down=-self.current_altitude, yaw=0.0, wait=3.0)

                    # Check if descent recovered tags
                    for _ in range(30):
                        rclpy.spin_once(self, timeout_sec=0.1)
                    if self.tags_visible():
                        self.get_logger().info('Tags recovered after descent')
                        tag_lost_since = None
                        # Re-send original waypoint at new altitude
                        self.send_command('goto_ned', north=north, east=east,
                                          down=-self.current_altitude, yaw=0.0, wait=0.5)
                        continue

                # Step 2: Retreat toward last good position
                retreat_north = self.last_good_north
                retreat_east = self.last_good_east
                self.get_logger().info(
                    f'Retreating to last good: NED({retreat_north:.1f}, {retreat_east:.1f})')
                self.send_command('goto_ned',
                                  north=retreat_north, east=retreat_east,
                                  down=-self.current_altitude, yaw=0.0, wait=5.0)

                # Wait and check for recovery
                for _ in range(50):
                    rclpy.spin_once(self, timeout_sec=0.1)
                if self.tags_visible():
                    self.get_logger().info('Tags recovered after retreat')
                    tag_lost_since = None
                    # Re-climb to cruise altitude and retry waypoint
                    self.current_altitude = self.flight_altitude
                    self.send_command('goto_ned', north=north, east=east,
                                      down=-self.current_altitude, yaw=0.0, wait=0.5)
                    continue

                # Step 3: Still no tags — land safely
                self.get_logger().error('Recovery failed — landing')
                return False

            # Re-send goto periodically to keep the command active
            if int((time.time() - start) * 10) % 50 == 0:
                self.send_command('goto_ned', north=north, east=east,
                                  down=-self.current_altitude, yaw=0.0, wait=0.1)

        self.get_logger().warn(f'Timeout reaching WP NED({north:.1f}, {east:.1f})')
        return False

    def execute(self):
        try:
            # Wait for position
            self.get_logger().info('Waiting for position data...')
            for _ in range(60):
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.position_valid:
                    break
            if not self.position_valid:
                self.get_logger().error('No position data')
                return

            self.get_logger().info(
                f'Position: NED({self.drone_x:.2f}, {self.drone_y:.2f}, {self.drone_z:.2f})')

            # Startup
            self.get_logger().info('=== Starting corridor navigation ===')
            self.send_command('start_offboard_heartbeat', wait=3.0)
            self.send_command('arm', wait=3.0)
            self.send_command('offboard_takeoff',
                              distance=self.flight_altitude, wait=8.0)

            # Wait for tag odometry to lock
            self.get_logger().info('Waiting for tag odometry lock...')
            lock_start = time.time()
            while time.time() - lock_start < 15.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.tags_visible():
                    self.get_logger().info('Tag odometry locked')
                    break
            else:
                self.get_logger().warn('No tag odometry — proceeding anyway')

            self.last_good_north = self.drone_x
            self.last_good_east = self.drone_y

            # Navigate through waypoints
            for i, (north, east) in enumerate(self.waypoints):
                self.get_logger().info(
                    f'=== WP {i+1}/{len(self.waypoints)}: NED({north:.1f}, {east:.1f}) ===')

                reached = self.goto_waypoint(north, east)
                if not reached:
                    self.get_logger().error('Navigation failed — aborting')
                    break

            self.get_logger().info('=== Corridor complete ===')

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted')
        finally:
            self.send_command('land', wait=8.0)
            self.send_command('disarm', wait=2.0)
            self.send_command('stop_offboard_heartbeat', wait=1.0)
            self.get_logger().info('Done')


def main(args=None):
    rclpy.init(args=args)
    node = CorridorNavigation()
    try:
        node.execute()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
