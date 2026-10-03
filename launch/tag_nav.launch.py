"""Launch tag_nav, the AprilTag navigation primitives behind /dexi/tag_nav/execute.

    ros2 launch dexi_apriltag tag_nav.launch.py                      # DEXI 5 defaults
    ros2 launch dexi_apriltag tag_nav.launch.py config:=tag_nav_sim.yaml
    ros2 launch dexi_apriltag tag_nav.launch.py dry_run:=true        # log, send nothing

Requires apriltag_node publishing tag TFs, the base_link->camera static transform
and the offboard manager (all in the DEXI bringup).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('dexi_apriltag')
    config = LaunchConfiguration('config')
    dry_run = LaunchConfiguration('dry_run')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='tag_nav_dexi5.yaml',
                              description='Per-airframe parameter file in dexi_apriltag/config'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='Compute and log, send nothing to the offboard manager'),
        # PX4 publishes vehicle_local_position at 100 Hz and every message wakes rclpy's
        # Python wait-set rebuild (83% of a CM4 core at idle). The offboard manager echoes
        # the stream at 20 Hz on /fmu/out/vehicle_local_position_20hz; tag_nav reads that.
        Node(
            package='dexi_apriltag',
            executable='tag_nav.py',
            name='tag_nav',
            output='screen',
            parameters=[
                PathJoinSubstitution([share, 'config', config]),
                {'dry_run': ParameterValue(dry_run, value_type=bool),
                 'local_position_topic': '/fmu/out/vehicle_local_position_20hz'},
            ],
        ),
    ])
