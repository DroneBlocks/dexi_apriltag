from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'base_link_frame',
        default_value='base_link',
        description='Base link frame for odometry'))

    ld.add_action(DeclareLaunchArgument(
        'tag_family',
        default_value='tag36h11',
        description='AprilTag family'))

    ld.add_action(DeclareLaunchArgument(
        'min_detection_quality',
        default_value='0.5',
        description='Minimum detection quality threshold (0.0-1.0)'))

    ld.add_action(DeclareLaunchArgument(
        'target_tag_id',
        default_value='-1',
        description='Target tag ID to use (-1 = use best quality tag)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_rect',
        description='Camera image topic'))

    ld.add_action(DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera_info',
        description='Camera info topic'))

    ld.add_action(DeclareLaunchArgument(
        'apriltag_family',
        default_value='36h11',
        description='AprilTag family for apriltag_ros'))

    ld.add_action(DeclareLaunchArgument(
        'apriltag_size',
        default_value='0.173',
        description='AprilTag size in meters'))

    # Static transform arguments for camera mounting
    ld.add_action(DeclareLaunchArgument(
        'camera_x',
        default_value='0.0',
        description='Camera X offset from base_link (meters, forward positive)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_y',
        default_value='0.0',
        description='Camera Y offset from base_link (meters, left positive)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_z',
        default_value='0.0',
        description='Camera Z offset from base_link (meters, up positive)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_roll',
        default_value='0.0',
        description='Camera roll rotation (radians)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_pitch',
        default_value='1.5708',
        description='Camera pitch rotation (radians, default: 1.5708 = 90° downward-facing)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_yaw',
        default_value='0.0',
        description='Camera yaw rotation (radians)'))

    # Get launch configuration values
    base_link_frame = LaunchConfiguration('base_link_frame')
    tag_family = LaunchConfiguration('tag_family')
    min_detection_quality = LaunchConfiguration('min_detection_quality')
    target_tag_id = LaunchConfiguration('target_tag_id')
    camera_topic = LaunchConfiguration('camera_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    apriltag_family = LaunchConfiguration('apriltag_family')
    apriltag_size = LaunchConfiguration('apriltag_size')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    # Static transform publisher: base_link -> camera
    # This defines where the camera is mounted on the drone
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

    # AprilTag Detection Node (apriltag_ros)
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
            'tag_family': apriltag_family,
            'tag_size': apriltag_size,
        }]
    )
    ld.add_action(apriltag_node)

    # AprilTag Odometry Node
    apriltag_odometry_node = Node(
        package='dexi_apriltag',
        executable='apriltag_odometry',
        name='apriltag_odometry',
        output='screen',
        parameters=[{
            'base_link_frame': base_link_frame,
            'tag_family': tag_family,
            'min_detection_quality': min_detection_quality,
            'target_tag_id': target_tag_id,
            'position_variance': [0.01, 0.01, 100.0],  # XY from vision, Z from distance sensor
            'orientation_variance': [0.01, 0.01, 0.01],
            'velocity_variance': [0.1, 0.1, 0.1],
        }]
    )
    ld.add_action(apriltag_odometry_node)

    return ld
