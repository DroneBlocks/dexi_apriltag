from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'following_enabled',
        default_value='true',
        description='Enable AprilTag following on startup'))

    ld.add_action(DeclareLaunchArgument(
        'target_altitude',
        default_value='2.0',
        description='Target altitude above ground in meters'))

    ld.add_action(DeclareLaunchArgument(
        'proportional_gain_xy',
        default_value='2.0',
        description='Proportional gain for XY movement'))

    ld.add_action(DeclareLaunchArgument(
        'proportional_gain_z',
        default_value='0.3',
        description='Proportional gain for Z movement'))

    ld.add_action(DeclareLaunchArgument(
        'max_velocity_xy',
        default_value='3.0',
        description='Maximum XY velocity in m/s'))

    ld.add_action(DeclareLaunchArgument(
        'max_velocity_z',
        default_value='1.0',
        description='Maximum Z velocity in m/s'))

    ld.add_action(DeclareLaunchArgument(
        'deadband_pixels',
        default_value='20.0',
        description='Pixel deadband for centering'))

    ld.add_action(DeclareLaunchArgument(
        'target_tag_id',
        default_value='0',
        description='Target tag ID to follow (0 = any tag)'))

    ld.add_action(DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization node'))

    ld.add_action(DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_rect',
        description='Camera image topic'))

    ld.add_action(DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera_info',
        description='Camera info topic'))

    ld.add_action(DeclareLaunchArgument(
        'tag_family',
        default_value='36h11',
        description='AprilTag family'))

    ld.add_action(DeclareLaunchArgument(
        'tag_size',
        default_value='0.173',
        description='AprilTag size in meters'))

    # Get launch configuration values
    following_enabled = LaunchConfiguration('following_enabled')
    target_altitude = LaunchConfiguration('target_altitude')
    proportional_gain_xy = LaunchConfiguration('proportional_gain_xy')
    proportional_gain_z = LaunchConfiguration('proportional_gain_z')
    max_velocity_xy = LaunchConfiguration('max_velocity_xy')
    max_velocity_z = LaunchConfiguration('max_velocity_z')
    deadband_pixels = LaunchConfiguration('deadband_pixels')
    target_tag_id = LaunchConfiguration('target_tag_id')
    enable_visualization = LaunchConfiguration('enable_visualization')
    camera_topic = LaunchConfiguration('camera_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    tag_family = LaunchConfiguration('tag_family')
    tag_size = LaunchConfiguration('tag_size')

    # AprilTag Detection Node (apriltag_ros)
    # Note: Running in global namespace (no namespace parameter) to match CM4 approach
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        remappings=[
            ('image_rect', camera_topic),
            ('camera_info', camera_info_topic),
            ('detections', '/detections'),
        ],
        parameters=[{
            'image_transport': 'raw',
            'tag_family': tag_family,
            'tag_size': tag_size,
        }]
    )
    ld.add_action(apriltag_node)

    # AprilTag Follower Node
    # Note: Running in global namespace to match CM4 approach
    # Subscribes to /detections (published by apriltag_node above)
    apriltag_follower_node = Node(
        package='dexi_apriltag',
        executable='apriltag_follower',
        name='apriltag_follower',
        output='screen',
        parameters=[{
            'following_enabled': following_enabled,
            'target_altitude': target_altitude,
            'proportional_gain_xy': proportional_gain_xy,
            'proportional_gain_z': proportional_gain_z,
            'max_velocity_xy': max_velocity_xy,
            'max_velocity_z': max_velocity_z,
            'deadband_pixels': deadband_pixels,
            'target_tag_id': target_tag_id,
        }]
    )
    ld.add_action(apriltag_follower_node)

    # AprilTag Visualizer Node (optional)
    # Note: Subscribes to /apriltag_detections (hardcoded in C++ code)
    # This doesn't match the /detections topic from apriltag_node, so we need a remap
    apriltag_visualizer_node = Node(
        package='dexi_apriltag',
        executable='apriltag_visualizer',
        name='apriltag_visualizer',
        output='screen',
        remappings=[
            ('/apriltag_detections', '/detections'),
        ],
        parameters=[{
            'publish_raw': False,
            'publish_compressed': True,
        }],
        condition=IfCondition(enable_visualization)
    )
    ld.add_action(apriltag_visualizer_node)

    return ld
