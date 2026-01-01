# dexi_apriltag

AprilTag detection and tracking for DEXI drones.

## Indoor Flight (Optical Flow)

For indoor drones using optical flow:

- **Precision landing** works with optical flow alone - no additional position source needed
- **apriltag_odometry** is only needed for extended hover where optical flow drift becomes a problem

Typical workflow:
1. Fly normally with optical flow
2. Run `precision_landing` when ready to land on a tag
3. Use `apriltag_odometry` only for drift-free position hold over a tag

## Nodes

### apriltag_odometry

Publishes AprilTag pose as visual odometry to PX4 EKF2 for drift-free position hold. The drone maintains its position when the tag is visible, eliminating optical flow drift. When the tag is lost, falls back to optical flow.

**Note:** This anchors the drone to the locked position. Flying to a new location while the tag is visible will cause the drone to fight the command (it thinks it's drifting). Best used for hover/position hold scenarios.

```bash
ros2 launch dexi_apriltag apriltag_odometry.launch.py
```

**EKF2 Setup** (run once via MAVLink console or QGC):
```bash
param set EKF2_EV_CTRL 1      # Vision horizontal position only
param set EKF2_HGT_REF 0      # Barometer for height
param set EKF2_GPS_CTRL 0     # Disable GPS (if not available)
```

### precision_landing

Autonomous precision landing on an AprilTag. Designed for body-fixed cameras with slow movement and filtering to prevent orbiting.

States: SEARCHING → DETECTED (5s wait) → CENTERING (1.5s stable lock) → LANDING → LANDED

LED feedback: Off → Purple → White → Red → Green

Features:
- Slow centering (10cm/s) to minimize camera tilt
- Moving average filter (5 samples) for smooth position tracking
- Stable lock required before descent (must stay centered 1.5s)
- Pauses `px4_offboard_manager` setpoints during landing

Parameters:
| Parameter | Default | Description |
|-----------|---------|-------------|
| `centering_threshold` | 0.25m | Distance from center to consider "centered" |
| `centering_speed` | 0.10 m/s | Horizontal movement speed during centering |
| `stable_centering_duration` | 1.5s | Time to hold center before descent |
| `detection_delay` | 5.0s | Wait time after first detecting tag |
| `descent_rate` | 0.3 m/s | Descent speed during landing |
| `filter_length` | 5 | Moving average filter samples |

```bash
# Land on tag ID 0 (default)
ros2 launch dexi_apriltag precision_landing.launch.py

# Land on a different tag ID
ros2 launch dexi_apriltag precision_landing.launch.py target_tag_id:=5
```

### apriltag_follower

Actively follows a moving AprilTag by publishing position setpoints to PX4. Use for tracking moving targets.

```bash
ros2 launch dexi_apriltag apriltag_follow.launch.py
```

## Parameters

Common parameters available via launch arguments:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tag_family` | tag36h11 | AprilTag family |
| `target_tag_id` | 0 | Tag ID to track |
| `publish_rate` | 10.0 | Odometry publish rate (Hz) |
| `position_variance` | [2.0, 2.0, 100.0] | Position variance for EKF2 |

## Dependencies

- `apriltag_ros` - AprilTag detection
- `px4_msgs` - PX4 message types
- `tf2_ros` - Transform lookups
