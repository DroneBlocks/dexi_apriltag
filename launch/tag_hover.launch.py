from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    """Build the node list once tag_id has been resolved to a concrete int.

    ROS2 launch substitutions can't fill array parameters, so the
    single-element arrays (sequence, tag_map_*) are constructed here in
    Python after `tag_id` has been read out of the launch context.
    """
    tag_id = int(LaunchConfiguration('tag_id').perform(context))

    sequence = [tag_id]
    tag_map_ids = [tag_id]
    tag_map_n = [0.0]
    tag_map_e = [0.0]

    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')
    enable_odometry = LaunchConfiguration('enable_odometry')
    odometry_publish_rate = LaunchConfiguration('odometry_publish_rate')
    hover_duration = LaunchConfiguration('hover_duration')
    centering_speed = LaunchConfiguration('centering_speed')
    centering_threshold = LaunchConfiguration('centering_threshold')
    tag_loss_grace = LaunchConfiguration('tag_loss_grace')
    min_takeoff_altitude = LaunchConfiguration('min_takeoff_altitude')
    detection_delay = LaunchConfiguration('detection_delay')
    detection_led_color = LaunchConfiguration('detection_led_color')
    lock_led_color = LaunchConfiguration('lock_led_color')
    tag_family = LaunchConfiguration('tag_family')

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_tf',
        arguments=[
            camera_x, camera_y, camera_z,
            camera_roll, camera_pitch, camera_yaw,
            'base_link', 'camera',
        ],
        output='screen',
    )

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
        condition=IfCondition(enable_odometry),
    )

    tag_hover_node = Node(
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
            'centering_speed': centering_speed,
            'centering_threshold': centering_threshold,
            'tag_loss_grace': tag_loss_grace,
            'min_takeoff_altitude': min_takeoff_altitude,
            'detection_delay': detection_delay,
            'detection_led_color': detection_led_color,
            'lock_led_color': lock_led_color,
        }],
    )

    return [static_tf_node, apriltag_odometry_node, tag_hover_node]


def generate_launch_description():
    """Launch a tag-hover (yaw-aligned, locked-on hold over a single tag).

    Drone takes off in POSCTL, flies over the target tag, tag_hop engages,
    yaw-aligns to the tag's printed up-edge (closed-loop), then holds for
    hover_duration. AUTO.LAND on completion.

    Quick start (default: hover over tag 0 for 10 minutes):
        ros2 launch dexi_apriltag tag_hover.launch.py

    Different tag or shorter hover:
        ros2 launch dexi_apriltag tag_hover.launch.py \\
            tag_id:=3 hover_duration:=60.0

    Disable apriltag_odometry (body-frame-only, accept EKF drift):
        ros2 launch dexi_apriltag tag_hover.launch.py enable_odometry:=false

    LED progression: off → purple (settle) → cyan (centering) → white
    (locked) → red (descent) → green (landed).
    """
    ld = LaunchDescription()

    # ----- Tag selection -----

    ld.add_action(DeclareLaunchArgument(
        'tag_id', default_value='0',
        description='AprilTag ID to hover over (integer)'))

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
                    'Highly recommended for hover — anchors EKF to the tag so '
                    'long holds don\'t accumulate position drift.'))
    ld.add_action(DeclareLaunchArgument(
        'odometry_publish_rate', default_value='10.0',
        description='apriltag_odometry publish rate (Hz)'))

    # ----- tag_hover scalar args -----

    ld.add_action(DeclareLaunchArgument(
        'hover_duration', default_value='600.0',
        description='Seconds to hold over the tag before AUTO.LAND (default 10 min)'))
    ld.add_action(DeclareLaunchArgument(
        'centering_speed', default_value='0.20',
        description='Body-frame velocity while chasing the tag in CENTERING (m/s)'))
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
        description='LED color during CENTERING (acquiring the tag)'))
    ld.add_action(DeclareLaunchArgument(
        'lock_led_color', default_value='white',
        description='LED color during HOLDING (locked on the tag)'))
    ld.add_action(DeclareLaunchArgument(
        'tag_family', default_value='tag36h11',
        description='AprilTag family'))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
