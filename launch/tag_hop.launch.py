from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch tag-hop corridor navigation.

    Brings up everything tag_hop needs (static TF, optional vision-corrected
    EKF, the tag_hop node itself). Assumes apriltag_ros is already running
    from dexi_bringup so that /tf is being populated for detected tags.

    Quick start (default corridor: tag 0 -> 2 -> 4 -> 2 -> 0, 2m spacing):
        ros2 launch dexi_apriltag tag_hop.launch.py

    Adjust scalar parameters:
        ros2 launch dexi_apriltag tag_hop.launch.py \\
            hover_duration:=5.0 transit_speed:=0.15

    Disable apriltag_odometry (body-frame-only test, accept hover drift):
        ros2 launch dexi_apriltag tag_hop.launch.py enable_odometry:=false

    For non-default corridor layouts (different sequence or tag spacing),
    copy this file and edit the sequence / tag_map_ids / tag_map_n / tag_map_e
    constants below. ROS2 launch substitutions don't support array params.
    """
    ld = LaunchDescription()

    # ----- Static transform args -----

    ld.add_action(DeclareLaunchArgument(
        'camera_x', default_value='0.0',
        description='Camera X offset from base_link (meters)'))
    ld.add_action(DeclareLaunchArgument(
        'camera_y', default_value='0.0',
        description='Camera Y offset from base_link (meters)'))
    ld.add_action(DeclareLaunchArgument(
        'camera_z', default_value='0.0',
        description='Camera Z offset from base_link (meters)'))
    ld.add_action(DeclareLaunchArgument(
        'camera_roll', default_value='0.0',
        description='Camera roll rotation (radians)'))
    ld.add_action(DeclareLaunchArgument(
        'camera_pitch', default_value='1.5708',
        description='Camera pitch rotation (radians, default: 90° downward)'))
    ld.add_action(DeclareLaunchArgument(
        'camera_yaw', default_value='0.0',
        description='Camera yaw rotation (radians)'))

    # ----- apriltag_odometry args -----

    ld.add_action(DeclareLaunchArgument(
        'enable_odometry', default_value='true',
        description='Run apriltag_odometry to feed VehicleOdometry into PX4 EKF2. '
                    'Strongly recommended — without it, NED-based HOLDING phases '
                    'drift physically as EKF runs on flow + IMU only.'))
    ld.add_action(DeclareLaunchArgument(
        'odometry_publish_rate', default_value='10.0',
        description='apriltag_odometry publish rate (Hz)'))

    # ----- tag_hop scalar args -----

    ld.add_action(DeclareLaunchArgument(
        'hover_duration', default_value='10.0',
        description='Seconds to hold over each tag'))
    ld.add_action(DeclareLaunchArgument(
        'transit_speed', default_value='0.20',
        description='Body-frame velocity during TRANSIT (m/s)'))
    ld.add_action(DeclareLaunchArgument(
        'centering_speed', default_value='0.20',
        description='Body-frame velocity while chasing a tag in CENTERING (m/s)'))
    ld.add_action(DeclareLaunchArgument(
        'min_transit_duration', default_value='1.0',
        description='Minimum seconds in TRANSIT before allowing tag acquisition'))
    ld.add_action(DeclareLaunchArgument(
        'transit_timeout', default_value='15.0',
        description='Max seconds in TRANSIT before handing off to PX4 AUTO.LAND'))
    ld.add_action(DeclareLaunchArgument(
        'centering_threshold', default_value='0.25',
        description='Body-frame distance to declare centered (m)'))
    ld.add_action(DeclareLaunchArgument(
        'tag_loss_grace', default_value='2.0',
        description='Seconds the tag can be lost before resetting CENTERING timers'))
    ld.add_action(DeclareLaunchArgument(
        'min_takeoff_altitude', default_value='0.30',
        description="Airborne gate — won't engage until above this (m)"))
    ld.add_action(DeclareLaunchArgument(
        'detection_delay', default_value='2.0',
        description='Settle window after first detection before engaging (s)'))
    ld.add_action(DeclareLaunchArgument(
        'detection_led_color', default_value='cyan',
        description='LED color while centering or holding on a tag'))
    ld.add_action(DeclareLaunchArgument(
        'tag_family', default_value='tag36h11',
        description='AprilTag family'))

    # ----- Array params (hardcoded — launch substitutions don't support arrays) -----
    # Default: 4m corridor with tags 0/2/4 along +N axis, full there-and-back sequence.
    # Edit these if your tag layout differs.

    sequence = [0, 2, 4, 2, 0]
    tag_map_ids = [0, 2, 4]
    tag_map_n = [0.0, 2.0, 4.0]
    tag_map_e = [0.0, 0.0, 0.0]

    # ----- Resolve LaunchConfigurations -----

    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')
    enable_odometry = LaunchConfiguration('enable_odometry')
    odometry_publish_rate = LaunchConfiguration('odometry_publish_rate')
    hover_duration = LaunchConfiguration('hover_duration')
    transit_speed = LaunchConfiguration('transit_speed')
    centering_speed = LaunchConfiguration('centering_speed')
    min_transit_duration = LaunchConfiguration('min_transit_duration')
    transit_timeout = LaunchConfiguration('transit_timeout')
    centering_threshold = LaunchConfiguration('centering_threshold')
    tag_loss_grace = LaunchConfiguration('tag_loss_grace')
    min_takeoff_altitude = LaunchConfiguration('min_takeoff_altitude')
    detection_delay = LaunchConfiguration('detection_delay')
    detection_led_color = LaunchConfiguration('detection_led_color')
    tag_family = LaunchConfiguration('tag_family')

    # ----- Static transform: base_link -> camera -----
    # Idempotent if dexi_bringup also publishes one with the same name.

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

    # ----- apriltag_odometry (optional) -----

    apriltag_odometry_node = Node(
        package='dexi_apriltag',
        executable='apriltag_odometry_node',
        name='apriltag_odometry',
        output='screen',
        parameters=[{
            'tag_family': tag_family,
            'publish_rate': odometry_publish_rate,
            'position_variance': [0.25, 0.25, 100.0],
            'orientation_variance': [0.01, 0.01, 0.01],
            'filter_length': 5,
            'tag_map_ids': tag_map_ids,
            'tag_map_x': tag_map_n,
            'tag_map_y': tag_map_e,
        }],
        condition=IfCondition(enable_odometry)
    )
    ld.add_action(apriltag_odometry_node)

    # ----- tag_hop -----

    tag_hop_node = Node(
        package='dexi_apriltag',
        executable='tag_hop.py',
        name='tag_hop',
        output='screen',
        parameters=[{
            'tag_family': tag_family,
            'sequence': sequence,
            'tag_map_ids': tag_map_ids,
            'tag_map_n': tag_map_n,
            'tag_map_e': tag_map_e,
            'hover_duration': hover_duration,
            'transit_speed': transit_speed,
            'centering_speed': centering_speed,
            'min_transit_duration': min_transit_duration,
            'transit_timeout': transit_timeout,
            'centering_threshold': centering_threshold,
            'tag_loss_grace': tag_loss_grace,
            'min_takeoff_altitude': min_takeoff_altitude,
            'detection_delay': detection_delay,
            'detection_led_color': detection_led_color,
        }]
    )
    ld.add_action(tag_hop_node)

    return ld
