from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch AprilTag visual odometry for PX4 EKF2 fusion.

    Note: Assumes apriltag_ros is already running (e.g., from dexi_bringup).
    """
    ld = LaunchDescription()

    # AprilTag arguments
    ld.add_action(DeclareLaunchArgument(
        'tag_family',
        default_value='tag36h11',
        description='AprilTag family'))

    ld.add_action(DeclareLaunchArgument(
        'target_tag_id',
        default_value='0',
        description='Target tag ID'))

    # Odometry arguments
    ld.add_action(DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Odometry publish rate in Hz (lower = more stable)'))

    # Static transform arguments for camera mounting
    ld.add_action(DeclareLaunchArgument(
        'camera_x',
        default_value='0.0',
        description='Camera X offset from base_link (meters)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_y',
        default_value='0.0',
        description='Camera Y offset from base_link (meters)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_z',
        default_value='0.0',
        description='Camera Z offset from base_link (meters)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_roll',
        default_value='0.0',
        description='Camera roll rotation (radians)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_pitch',
        default_value='1.5708',
        description='Camera pitch rotation (radians, default: 90 degrees downward)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_yaw',
        default_value='0.0',
        description='Camera yaw rotation (radians)'))

    # Get launch configuration values
    tag_family = LaunchConfiguration('tag_family')
    target_tag_id = LaunchConfiguration('target_tag_id')
    publish_rate = LaunchConfiguration('publish_rate')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    # Static transform publisher: base_link -> camera
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

    # AprilTag Odometry Node
    # Note: Array parameters must be hardcoded - LaunchConfiguration doesn't support arrays
    apriltag_odometry_node = Node(
        package='dexi_apriltag',
        executable='apriltag_odometry.py',
        name='apriltag_odometry',
        output='screen',
        parameters=[{
            'tag_family': tag_family,
            'target_tag_id': target_tag_id,
            'publish_rate': publish_rate,
            'position_variance': [2.0, 2.0, 100.0],  # Higher variance = less aggressive corrections
            'orientation_variance': [0.01, 0.01, 0.01],
            'filter_length': 5,  # Moving average filter (ModalAI approach)
        }]
    )
    ld.add_action(apriltag_odometry_node)

    return ld
