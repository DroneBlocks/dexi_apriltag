from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch precision landing with static transform.

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
        description='Target tag ID (0 = any tag)'))

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
        description='Camera pitch rotation (radians, default: 90° downward)'))

    ld.add_action(DeclareLaunchArgument(
        'camera_yaw',
        default_value='0.0',
        description='Camera yaw rotation (radians)'))

    # Precision landing arguments
    ld.add_action(DeclareLaunchArgument(
        'detection_delay',
        default_value='5.0',
        description='Seconds to wait after tag detection before landing'))

    ld.add_action(DeclareLaunchArgument(
        'centering_threshold',
        default_value='0.25',
        description='Distance threshold (m) to consider centered'))

    ld.add_action(DeclareLaunchArgument(
        'centering_speed',
        default_value='0.10',
        description='Horizontal centering speed (m/s)'))

    ld.add_action(DeclareLaunchArgument(
        'stable_centering_duration',
        default_value='1.5',
        description='Seconds to hold center before descent'))

    ld.add_action(DeclareLaunchArgument(
        'filter_length',
        default_value='5',
        description='Moving average filter samples'))

    ld.add_action(DeclareLaunchArgument(
        'descent_rate',
        default_value='0.3',
        description='Descent rate (m/s)'))

    ld.add_action(DeclareLaunchArgument(
        'final_descent_rate',
        default_value='0.15',
        description='Slower descent rate near ground (m/s)'))

    ld.add_action(DeclareLaunchArgument(
        'landing_altitude',
        default_value='0.15',
        description='Altitude threshold to detect landing (m)'))

    # Get launch configuration values
    tag_family = LaunchConfiguration('tag_family')
    target_tag_id = LaunchConfiguration('target_tag_id')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')
    detection_delay = LaunchConfiguration('detection_delay')
    centering_threshold = LaunchConfiguration('centering_threshold')
    centering_speed = LaunchConfiguration('centering_speed')
    stable_centering_duration = LaunchConfiguration('stable_centering_duration')
    filter_length = LaunchConfiguration('filter_length')
    descent_rate = LaunchConfiguration('descent_rate')
    final_descent_rate = LaunchConfiguration('final_descent_rate')
    landing_altitude = LaunchConfiguration('landing_altitude')

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

    # Precision Landing Node
    precision_landing_node = Node(
        package='dexi_apriltag',
        executable='precision_landing.py',
        name='precision_landing',
        output='screen',
        parameters=[{
            'tag_family': tag_family,
            'target_tag_id': target_tag_id,
            'detection_delay': detection_delay,
            'centering_threshold': centering_threshold,
            'centering_speed': centering_speed,
            'stable_centering_duration': stable_centering_duration,
            'filter_length': filter_length,
            'descent_rate': descent_rate,
            'final_descent_rate': final_descent_rate,
            'landing_altitude': landing_altitude,
        }]
    )
    ld.add_action(precision_landing_node)

    return ld
