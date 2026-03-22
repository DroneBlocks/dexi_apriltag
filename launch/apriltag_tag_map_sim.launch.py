import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch AprilTag odometry with tag map for simulation testing.

    Loads a tag map YAML and starts the odometry node in tag map mode.
    Assumes apriltag_ros is already running (e.g., from dexi_bringup_unity_sim).
    """
    ld = LaunchDescription()

    # Tag map file argument
    default_tag_map = os.path.join(
        get_package_share_directory('dexi_apriltag'),
        'config', 'tag_map_sim.yaml')

    ld.add_action(DeclareLaunchArgument(
        'tag_map_file',
        default_value=default_tag_map,
        description='Path to tag map YAML file'))

    # Camera mount arguments
    ld.add_action(DeclareLaunchArgument(
        'camera_x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument(
        'camera_y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument(
        'camera_z', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument(
        'camera_roll', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument(
        'camera_pitch', default_value='1.5708'))
    ld.add_action(DeclareLaunchArgument(
        'camera_yaw', default_value='0.0'))

    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    # Static transform: base_link -> camera
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_tf',
        arguments=[
            camera_x, camera_y, camera_z,
            camera_roll, camera_pitch, camera_yaw,
            'base_link', 'camera'
        ],
        output='screen'
    )
    ld.add_action(static_tf_node)

    # Load tag map YAML and extract arrays for ROS parameters
    # We need to resolve the default at generation time since
    # LaunchConfiguration isn't available yet for file I/O
    tag_map_file = LaunchConfiguration('tag_map_file')

    # For the default case, load the YAML directly
    tag_map_ids = []
    tag_map_x = []
    tag_map_y = []
    try:
        with open(default_tag_map, 'r') as f:
            tag_data = yaml.safe_load(f)
        tag_map = tag_data.get('tag_map', {})
        tag_map_ids = tag_map.get('ids', [])
        tag_map_x = tag_map.get('x', [])
        tag_map_y = tag_map.get('y', [])
    except Exception as e:
        print(f'Warning: Could not load tag map: {e}')

    # AprilTag Odometry Node with tag map
    apriltag_odometry_node = Node(
        package='dexi_apriltag',
        executable='apriltag_odometry.py',
        name='apriltag_odometry',
        output='screen',
        parameters=[{
            'tag_family': 'tag36h11',
            'publish_rate': 10.0,
            'position_variance': [2.0, 2.0, 100.0],
            'orientation_variance': [0.01, 0.01, 0.01],
            'filter_length': 5,
            'tag_map_ids': tag_map_ids,
            'tag_map_x': [float(v) for v in tag_map_x],
            'tag_map_y': [float(v) for v in tag_map_y],
        }]
    )
    ld.add_action(apriltag_odometry_node)

    return ld
