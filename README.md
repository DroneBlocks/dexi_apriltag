# dexi_apriltag

AprilTag detection and following package for DEXI drones.

## Overview

This package provides AprilTag-based visual tracking and following capabilities for PX4-based drones. It has been separated from the main `dexi_offboard` package to isolate computer vision dependencies and enable independent development.

## Nodes

### apriltag_follower

Visual servoing control node that commands the drone to follow detected AprilTags.

**Published Topics:**
- `/fmu/in/offboard_control_mode` - Offboard control mode heartbeat
- `/fmu/in/trajectory_setpoint` - Position setpoints for PX4
- `apriltag_visual_offset` - Debug topic showing pixel offset from image center

**Subscribed Topics:**
- `/detections` - AprilTag detections from apriltag_ros
- `/camera_info` - Camera calibration parameters
- `/fmu/out/vehicle_local_position` - Current drone position

**Parameters:**
- `following_enabled` (bool, default: true) - Enable/disable active following
- `target_altitude` (double, default: 2.0) - Target altitude in meters
- `proportional_gain_xy` (double, default: 2.0) - P gain for XY movement
- `proportional_gain_z` (double, default: 0.3) - P gain for altitude
- `max_velocity_xy` (double, default: 3.0) - Max horizontal velocity (m/s)
- `max_velocity_z` (double, default: 1.0) - Max vertical velocity (m/s)
- `deadband_pixels` (double, default: 20.0) - Centering tolerance in pixels
- `target_tag_id` (int, default: 0) - Specific tag ID to follow (0 = any tag)
- `detection_timeout` (double, default: 10.0) - Timeout for lost tag detections (seconds)

### apriltag_visualizer

Visualization node that overlays AprilTag detection information on camera images.

**Published Topics:**
- `apriltag_debug_image` - Raw debug image with overlays
- `apriltag_debug_image/compressed` - Compressed debug image

**Subscribed Topics:**
- `/cam0/image_raw/compressed` - Camera image stream
- `/dexi/apriltag_visual_offset` - Follower offset information
- `/apriltag_detections` - AprilTag detection results

**Parameters:**
- `publish_raw` (bool, default: true) - Publish uncompressed debug images
- `publish_compressed` (bool, default: true) - Publish compressed debug images

## Launch Files

### apriltag_follow.launch.py

Launches the AprilTag detection node (apriltag_ros), follower, and visualizer with configurable parameters.

**Basic Usage:**
```bash
# Default launch (following enabled, standard parameters)
ros2 launch dexi_apriltag apriltag_follow.launch.py

# Disable following (detection only)
ros2 launch dexi_apriltag apriltag_follow.launch.py following_enabled:=false

# Adjust velocity limits for faster tags
ros2 launch dexi_apriltag apriltag_follow.launch.py \
  max_velocity_xy:=2.0 \
  max_velocity_z:=1.0

# High-performance following for fast-moving tags
ros2 launch dexi_apriltag apriltag_follow.launch.py \
  proportional_gain_xy:=2.0 \
  max_velocity_xy:=3.0 \
  max_velocity_z:=1.0
```

**Available Parameters:**
- `following_enabled` (default: true) - Enable AprilTag following
- `target_altitude` (default: 2.0) - Target altitude in meters
- `proportional_gain_xy` (default: 2.0) - Horizontal movement responsiveness
- `proportional_gain_z` (default: 0.3) - Vertical movement responsiveness
- `max_velocity_xy` (default: 3.0) - Max horizontal speed (m/s)
- `max_velocity_z` (default: 1.0) - Max vertical speed (m/s)
- `deadband_pixels` (default: 20.0) - Centering tolerance in pixels
- `target_tag_id` (default: 0) - Specific tag ID to follow (0 = any tag)
- `enable_visualization` (default: true) - Enable debug visualization
- `camera_topic` (default: /image_rect) - Camera image topic
- `camera_info_topic` (default: /camera_info) - Camera calibration topic
- `tag_family` (default: 36h11) - AprilTag family
- `tag_size` (default: 0.173) - Physical tag size in meters

## Unity Simulation Setup

When using with Unity simulator:

1. **Start Unity and enable camera publishing**
2. **Decompress Unity's camera feed:**
   ```bash
   ros2 run image_transport republish compressed raw \
     --ros-args \
     -r in/compressed:=/cam0/image_raw/compressed \
     -r out:=/image_rect
   ```
3. **Publish camera_info** (if not provided by Unity)
4. **Launch AprilTag following:**
   ```bash
   ros2 launch dexi_apriltag apriltag_follow.launch.py
   ```

## Performance Tuning

### For Fast-Moving Tags
If the drone can't keep up with the tag:
- Increase `max_velocity_xy` (e.g., 2.0-3.0 m/s)
- Increase `proportional_gain_xy` (e.g., 1.5-2.0)
- Increase `max_velocity_z` (e.g., 1.0 m/s)

### For Smooth Following
For smoother, less aggressive tracking:
- Decrease `proportional_gain_xy` (e.g., 0.3-0.5)
- Decrease `max_velocity_xy` (e.g., 0.3-0.5 m/s)

### For Precise Centering
- Decrease `deadband_pixels` (e.g., 10-15 pixels)
- Increase `proportional_gain_xy`

## Dependencies

This package requires:
- `cv_bridge` - OpenCV-ROS bridge
- `image_transport` - Image transport plugins
- `apriltag_ros` - AprilTag detection library
- `apriltag_msgs` - AprilTag detection messages
- `px4_msgs` - PX4 autopilot messages
- Standard ROS 2 packages (rclcpp, tf2, geometry_msgs, sensor_msgs)

## Architecture Notes

The `apriltag_follower` node directly publishes PX4 control commands and operates independently of the `dexi_offboard` package. This allows it to control the drone without going through the offboard manager, enabling dedicated visual servoing control loops.

**Important:** The follower must be used with the drone already in OFFBOARD mode. It does not handle arming or mode switching - use `dexi_offboard` package for flight mode management.
