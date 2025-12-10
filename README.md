# dexi_apriltag

AprilTag detection and following package for DEXI drones.

## Overview

This package provides AprilTag-based visual tracking and following capabilities for PX4-based drones. It has been separated from the main `dexi_offboard` package to isolate computer vision dependencies and enable independent development.

### Two Modes of Operation

**1. Position Hold Mode (apriltag_odometry)**
- **Use when**: You want drift-free position hold using AprilTags as fixed landmarks
- **Behavior**: Drone maintains its current position, tag prevents optical flow drift
- **Tag role**: Fixed position reference (like GPS coordinates)
- **Output**: Visual odometry to PX4 EKF2 for sensor fusion
- **Example**: Hovering over landing pad, indoor navigation with tag map

**2. Following Mode (apriltag_follower)**
- **Use when**: You want the drone to actively track and follow a moving tag
- **Behavior**: Drone follows tag movement, maintaining relative position
- **Tag role**: Moving target to track
- **Output**: Direct position setpoints to PX4
- **Example**: Following a person, tracking a moving vehicle

## Nodes

### apriltag_odometry

Visual odometry node that publishes AprilTag-based position estimates to PX4 for sensor fusion with optical flow and other sensors.

**Published Topics:**
- `/fmu/in/vehicle_visual_odometry` - VehicleOdometry messages for PX4 EKF2 fusion

**Subscribed Topics:**
- `/detections` - AprilTag detections from apriltag_ros

**Parameters:**
- `base_link_frame` (string, default: "base_link") - Base link frame name
- `tag_family` (string, default: "tag36h11") - AprilTag family name
- `min_detection_quality` (double, default: 0.5) - Minimum quality threshold (0.0-1.0)
- `target_tag_id` (int, default: -1) - Specific tag ID to use (-1 = use best quality)
- `position_variance` (array, default: [0.01, 0.01, 100.0]) - Position variance in m² (XY low, Z high)
- `orientation_variance` (array, default: [0.01, 0.01, 0.01]) - Orientation variance in rad²
- `velocity_variance` (array, default: [0.1, 0.1, 0.1]) - Velocity variance in m²/s²

**Use Case:**
Use this node when you want the drone to maintain stable position hold without drift by using AprilTags as fixed position references. The drone will maintain its position relative to the tag, eliminating optical flow drift. Works seamlessly with optical flow and distance sensor - when tag is visible, XY position locks to tag; height always comes from distance sensor; when tag is lost, falls back to optical flow for XY.

**Sensor Fusion:**
- **XY Position**: AprilTag (drift-free) → Optical Flow (fallback when tag lost)
- **Z Position (Height)**: Distance sensor (always primary)
- **Yaw**: AprilTag (when visible) → IMU (fallback)

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

### apriltag_odometry.launch.py

Launches the AprilTag detection node (apriltag_ros) and odometry node for drift-free position hold.

**Basic Usage:**
```bash
# Default launch - downward-facing camera at drone center
ros2 launch dexi_apriltag apriltag_odometry.launch.py

# Camera mounted 10cm forward, 3cm down from center
ros2 launch dexi_apriltag apriltag_odometry.launch.py \
  camera_x:=0.1 \
  camera_z:=-0.03

# Use specific tag ID only
ros2 launch dexi_apriltag apriltag_odometry.launch.py target_tag_id:=5

# Adjust quality threshold
ros2 launch dexi_apriltag apriltag_odometry.launch.py min_detection_quality:=0.7
```

**Available Parameters:**
- `base_link_frame` (default: "base_link") - Base link frame
- `tag_family` (default: "tag36h11") - Tag family for TF lookups
- `min_detection_quality` (default: 0.5) - Minimum quality threshold
- `target_tag_id` (default: -1) - Specific tag ID (-1 = best quality)
- `camera_topic` (default: /image_rect) - Camera image topic
- `camera_info_topic` (default: /camera_info) - Camera calibration topic
- `apriltag_family` (default: 36h11) - AprilTag family for detection
- `apriltag_size` (default: 0.173) - Physical tag size in meters
- `camera_x`, `camera_y`, `camera_z` (default: 0.0) - Camera position offset in meters
- `camera_roll`, `camera_yaw` (default: 0.0) - Camera rotation in radians
- `camera_pitch` (default: 1.5708) - Camera pitch in radians (default: downward-facing)

**Setup Requirements:**
1. **Camera Mounting Configuration**: The launch file automatically publishes the `base_link → camera` static transform with the camera **downward-facing by default** (`camera_pitch:=1.5708`). Only adjust if your camera is mounted differently:
   ```bash
   # Default: Camera at drone center, facing down (no parameters needed)
   ros2 launch dexi_apriltag apriltag_odometry.launch.py

   # Camera offset from center (10cm forward, 5cm down)
   ros2 launch dexi_apriltag apriltag_odometry.launch.py \
     camera_x:=0.1 \
     camera_z:=-0.05

   # Forward-facing camera (change pitch to 0)
   ros2 launch dexi_apriltag apriltag_odometry.launch.py \
     camera_pitch:=0.0
   ```

2. **PX4 Configuration**: Enable vision position fusion
   ```bash
   param set EKF2_EV_CTRL 9       # Enable XY position + yaw fusion (not height)
   param set EKF2_HGT_MODE 2      # Use distance sensor for height
   param set EKF2_EV_DELAY 50     # Vision delay in ms
   param save
   reboot
   ```

   **Note**: `EKF2_EV_CTRL 9` enables horizontal position (XY) and yaw from AprilTag vision, while height (Z) comes from your distance sensor. This prevents conflicts between vision-based height and distance sensor height.

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
4. **Launch AprilTag node** (choose based on use case):

   **For position hold (odometry):**
   ```bash
   ros2 launch dexi_apriltag apriltag_odometry.launch.py \
     apriltag_size:=<YOUR_TAG_SIZE>
   ```

   **For following mode:**
   ```bash
   ros2 launch dexi_apriltag apriltag_follow.launch.py
   ```

**Important Unity Configuration:**
- Set camera principal point to image center: `(width/2, height/2)`
- Ensure tag size parameter matches Unity tag scale (in meters)
- For 320x240 camera: Principal Point X=160, Y=120

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

## Debugging

### Verify TF Tree
```bash
# Check what frames are being published
ros2 run tf2_ros tf2_echo base_link camera

# Visualize the full TF tree
ros2 run tf2_tools view_frames
# Opens frames.pdf showing the transform tree

# Check if tag is being detected
ros2 topic echo /detections

# Monitor odometry output
ros2 topic echo /fmu/in/vehicle_visual_odometry
```

### Common Issues

**Error: "base_link does not exist"**
- The launch file should automatically publish `base_link → camera` transform
- Verify it's running: `ros2 run tf2_ros tf2_echo base_link camera`
- If not working, check the static_transform_publisher is running: `ros2 node list | grep base_link_to_camera_tf`

**No odometry being published:**
- Check tags are detected: `ros2 topic echo /detections`
- Verify camera topics: `ros2 topic list | grep -E "image|camera_info"`
- Check apriltag_ros is running: `ros2 node list | grep apriltag`

**Position jumps or jitter:**
- Reduce trust in vision: increase `position_variance` to `[0.05, 0.05, 100.0]`
- Increase `min_detection_quality` to use only high-quality detections
- Check tag size parameter matches physical tag size

## Dependencies

This package requires:
- `cv_bridge` - OpenCV-ROS bridge
- `image_transport` - Image transport plugins
- `apriltag_ros` - AprilTag detection library
- `apriltag_msgs` - AprilTag detection messages
- `px4_msgs` - PX4 autopilot messages
- Standard ROS 2 packages (rclcpp, tf2, geometry_msgs, sensor_msgs)

## Architecture Notes

### apriltag_odometry
The `apriltag_odometry` node integrates with PX4's EKF2 sensor fusion by publishing `VehicleOdometry` messages. It works alongside other sensors (optical flow, distance sensor, IMU) and does not directly control the drone. The PX4 position controller uses the fused state estimate for position hold.

**Key design decisions:**
- Uses TF lookups instead of parsing detection messages for robustness
- High Z variance (100.0) ensures distance sensor is primary height source
- Quality-based tag selection for multi-tag scenarios
- Single tag approach (easily extensible to multi-tag fusion)

### apriltag_follower
The `apriltag_follower` node directly publishes PX4 control commands and operates independently of the `dexi_offboard` package. This allows it to control the drone without going through the offboard manager, enabling dedicated visual servoing control loops.

**Important:** The follower must be used with the drone already in OFFBOARD mode. It does not handle arming or mode switching - use `dexi_offboard` package for flight mode management.

### Node Separation
These two nodes serve different purposes and should not be run simultaneously:
- **Run odometry** for drift-free position hold with tag as fixed reference
- **Run follower** for actively tracking and following a moving tag
