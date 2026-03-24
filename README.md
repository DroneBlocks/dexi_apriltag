# dexi_apriltag

AprilTag detection and tracking for DEXI drones.

## Simulation: Tag Map Navigation (PX4 SITL + Unity)

Navigate between AprilTag waypoints using tag-based visual odometry fused into PX4's EKF2.

### Required PX4 Parameters

These must be set after each PX4 SITL restart (they don't persist):

```bash
# Enable external vision position fusion in EKF2
param set EKF2_EV_CTRL 15       # Fuse EV position + velocity + yaw + height (SITL only)

# Precision indoor flight speeds
param set MPC_XY_VEL_MAX 0.5    # Max horizontal velocity (m/s)
param set MPC_XY_CRUISE 0.25    # Cruise speed (m/s)
```

To set these from the host:
```bash
docker exec dexi-sim-ftw-px4-sitl-1 /opt/px4/bin/px4-param set EKF2_EV_CTRL 15
docker exec dexi-sim-ftw-px4-sitl-1 /opt/px4/bin/px4-param set MPC_XY_VEL_MAX 0.5
docker exec dexi-sim-ftw-px4-sitl-1 /opt/px4/bin/px4-param set MPC_XY_CRUISE 0.25
```

### Running the Simulation

**Step 1: Start the odometry pipeline** (in the ros2-dev container)

```bash
# Static transform: base_link -> camera (downward-facing, 90° pitch)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 1.5708 0 base_link camera &

# AprilTag odometry node with tag map
python3 /home/ubuntu/dexi_ws/src/dexi_apriltag/scripts/apriltag_odometry.py \
  --ros-args \
  -p tag_family:=tag36h11 \
  -p publish_rate:=10.0 \
  -p 'position_variance:=[2.0,2.0,100.0]' \
  -p filter_length:=5 \
  -p 'tag_map_ids:=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]' \
  -p 'tag_map_x:=[0.0,0.0,1.0,1.0,2.0,2.0,3.0,3.0,4.0,4.0,5.0,5.0,6.0,6.0,7.0,7.0,8.0,8.0,9.0,9.0]' \
  -p 'tag_map_y:=[-0.25,0.25,-0.25,0.25,-0.25,0.25,-0.25,0.25,-0.25,0.25,-0.25,0.25,-0.25,0.25,-0.25,0.25,-0.25,0.25,-0.25,0.25]'
```

**Step 2: Start Unity** — Open `AprilTagNavigationScene` and hit Play

**Step 3: Verify EKF2 fusion**

```bash
# Should show cs_ev_pos: True
ros2 topic echo /fmu/out/estimator_status_flags --field cs_ev_pos --once
```

**Step 4: Run the shuttle test**

```bash
# Fly from tag 0 (NED origin) to tag 10 (NED North=5m) and back
python3 /home/ubuntu/dexi_ws/src/dexi_apriltag/scripts/tag_map_shuttle_test.py \
  --ros-args \
  -p target_north:=5.0 \
  -p target_east:=0.0 \
  -p flight_altitude:=2.0 \
  -p position_threshold:=0.25
```

### Tag Map Configuration

The tag map is defined in `config/tag_map_sim.yaml`. It maps tag IDs to NED world coordinates:

- **x**: NED North position (meters) — corresponds to Unity Z axis
- **y**: NED East position (meters) — corresponds to Unity X axis

The Unity `AprilTagNavigationScene` has a 2×10 grid of 6-inch (0.15m) tags in simulation (4-inch / 0.10m for real hardware):
- 2 columns (X = -0.25m, +0.25m), 10 rows (Z = 0m to 9m)
- Tag IDs 0-19, spacing: 0.5m across, 1.0m along flight path
- Flight area: 15ft × 15ft × 40ft cage

### Navigate and Precision Land

Fly from start to a destination tag and precision land on it:

```bash
# Fly to NED(9,0) at end of tag strip, then precision land on tag 18
python3 /home/ubuntu/dexi_ws/src/dexi_apriltag/scripts/tag_navigate_and_land.py \
  --ros-args \
  -p target_north:=9.0 \
  -p target_east:=0.0 \
  -p flight_altitude:=2.0 \
  -p landing_tag_id:=18

# Fly to halfway point and land on tag 10
python3 /home/ubuntu/dexi_ws/src/dexi_apriltag/scripts/tag_navigate_and_land.py \
  --ros-args \
  -p target_north:=5.0 \
  -p target_east:=0.0 \
  -p flight_altitude:=2.0 \
  -p landing_tag_id:=10
```

Phases: Takeoff → Navigate to destination (tag odometry + EKF2) → Hover → Precision land on target tag (centering + descent)

### Typical Results

| Speed | Position Accuracy | Lateral Drift | Notes |
|-------|------------------|---------------|-------|
| 1.0 m/s | ~0.25m | ±0.15m | Fast, occasional timeouts |
| 0.5 m/s | ~0.16m | ±0.09m | Good balance of speed and accuracy |
| 0.25 m/s | **~0.10m** | **±0.05m** | Best accuracy, recommended for precision work |

Tags visible at 2m altitude: 4 simultaneously

### Speed vs Accuracy Tradeoff

Slower speeds give the tag detector more clean frames and the position controller more time to converge. For real hardware, start at 0.25 m/s and increase as confidence grows.

## Real Hardware Setup (Optical Flow + Range + AprilTags)

For DEXI with optical flow, range sensor, and AprilTag visual odometry fused together.

### PX4 Parameters

```bash
# Sensor fusion
EKF2_EV_CTRL      1       # Vision horizontal position only (tags handle XY, not height/yaw)
EKF2_HGT_REF      2       # Range sensor as primary height source
EKF2_RNG_CTRL     2       # Always fuse range sensor
EKF2_OF_CTRL      1       # Enable optical flow
EKF2_OF_QMIN      50      # Lower flow quality threshold for indoors
EKF2_GPS_CTRL     0       # Disable GPS (indoor)
COM_ARM_WO_GPS    1       # Allow arming without GPS

# Flight speeds (precision indoor)
MPC_XY_VEL_MAX    0.5     # Max horizontal velocity (m/s)
MPC_XY_CRUISE     0.25    # Cruise speed (m/s)
```

### How the three sensors complement each other

| Source | Provides | Rate | Weakness |
|--------|----------|------|----------|
| **Optical flow** | Velocity (high rate) | ~50Hz | No absolute position, drifts over time |
| **Range sensor** | Altitude | ~50Hz | Only Z axis |
| **AprilTag odometry** | Absolute XY position | ~10-15Hz | Only works when tags visible |

- Between tag detections, optical flow keeps the estimate smooth
- When a tag is seen, it corrects accumulated drift (absolute position fix)
- Range sensor keeps altitude locked independently
- `EKF2_EV_CTRL=1` (not 15) tells EKF2 to only fuse tag XY position — optical flow handles velocity, range sensor handles height

### SITL vs Hardware Parameter Differences

| Parameter | SITL | Hardware | Why |
|-----------|------|----------|-----|
| `EKF2_EV_CTRL` | **15** | **1** | SITL has no real flow/range — tags provide everything. Hardware: tags provide XY position only |
| `EKF2_HGT_REF` | default | **2** (Range) | Hardware uses real range sensor for altitude |
| `EKF2_RNG_CTRL` | default | **2** (Always) | Always fuse range sensor on hardware |
| `EKF2_OF_CTRL` | default | **1** | Enable real optical flow on hardware |
| `EKF2_GPS_CTRL` | default | **0** | No GPS indoors |

## Indoor Flight (Optical Flow + Range Sensor)

DEXI uses three complementary sensors for indoor flight:

| Sensor | Role | PX4 Parameter |
|--------|------|---------------|
| **Range sensor** | Altitude (primary height) | `EKF2_HGT_REF=2`, `EKF2_RNG_CTRL=2` |
| **Optical flow** | Velocity / short-term position | `EKF2_OF_CTRL=1` |
| **AprilTag vision** | Absolute horizontal position (drift correction) | `EKF2_EV_CTRL=1` |

Optical flow gives good velocity estimates but drifts over time (no absolute position reference). AprilTag odometry corrects this drift by providing an absolute position fix when a tag is visible. The range sensor handles altitude independently.

Typical workflow:
1. Fly normally with optical flow + range sensor
2. Use `apriltag_odometry` for drift-free position hold over a tag
3. Run `precision_landing` when ready to land on a tag

## Nodes

### apriltag_odometry

Publishes AprilTag pose as visual odometry to PX4 EKF2 for drift-free position hold. The drone maintains its position when the tag is visible, eliminating optical flow drift. When the tag is lost, falls back to optical flow.

Supports two modes:
- **Single tag mode** (default) — one tag as implicit origin
- **Tag map mode** — multiple tags at known world positions (see `apriltag_tag_map_sim.launch.py`)

**Note:** This anchors the drone to the locked position. Flying to a new location while the tag is visible will cause the drone to fight the command (it thinks it's drifting). Best used for hover/position hold scenarios.

```bash
# Single tag mode
ros2 launch dexi_apriltag apriltag_odometry.launch.py target_tag_id:=0

# Tag map mode (multiple tags at known positions)
ros2 launch dexi_apriltag apriltag_tag_map_sim.launch.py
```

**EKF2 Setup** (run once via MAVLink console or QGC):
```bash
param set EKF2_EV_CTRL 1      # Vision horizontal position only
param set EKF2_HGT_REF 2      # Range sensor for height
param set EKF2_RNG_CTRL 2     # Always fuse range sensor
param set EKF2_OF_CTRL 1      # Enable optical flow
param set EKF2_GPS_CTRL 0     # Disable GPS
```

**Testing before flight (hand-held dry run):**

Use `dry_run` mode to verify the pipeline without publishing to the EKF2. Hold the drone by hand over a tag and move it around — the position estimates should track your movement.

```bash
sudo bash -c 'source /home/dexi/ros2_jazzy/install/setup.bash && \
  source /home/dexi/dexi_ws/install/setup.bash && \
  ros2 launch dexi_apriltag apriltag_odometry.launch.py target_tag_id:=0 dry_run:=true'
```

Output shows body-frame offset and NED position at ~1Hz:
```
Origin locked - heading: -56.8°, offset: [0.11, 0.14, 0.33]
Body: [0.08, -0.13] | NED: [0.04, -0.00] | Hdg: 303°
```

Move the drone left/right/forward/back — the Body and NED values should track your movement. If values stay constant, check that `/tf` is publishing and the tag is in view.

**Flight test procedure:**

The odometry node can be started before or during flight. It waits for a tag to become visible, then locks its origin to the EKF2's current position — no jump or discontinuity.

*Step 1: Verify PX4 parameters* (one-time setup via QGC MAVLink console)

```
param set EKF2_EV_CTRL 1      # Vision horizontal position
param set EKF2_HGT_REF 2      # Range sensor for height
param set EKF2_RNG_CTRL 2     # Always fuse range sensor
param set EKF2_OF_CTRL 1      # Enable optical flow
param set EKF2_GPS_CTRL 0     # Disable GPS
param set EKF2_OF_QMIN 50     # Lower flow quality threshold for indoors
param set COM_ARM_WO_GPS 1    # Allow arming without GPS
```

*Step 2: Start the odometry node* (SSH terminal 1)

```bash
sudo bash -c 'source /home/dexi/ros2_jazzy/install/setup.bash && \
  source /home/dexi/dexi_ws/install/setup.bash && \
  ros2 launch dexi_apriltag apriltag_odometry.launch.py target_tag_id:=0'
```

The node will print `Waiting for heading/position/attitude...` until the FC is publishing, then wait for a tag detection.

*Step 3: Take off with optical flow*

Take off normally using the offboard manager or RC. Optical flow + range sensor handle position and altitude. The odometry node is running but idle (no tag in view from altitude).

*Step 4: Fly over a tag*

When the camera sees the tag, the odometry node logs:
```
Tag detected - starting odometry publishing
Origin locked - heading: -56.8°, offset: [0.11, 0.14, 0.33]
Body: [0.08, -0.13] | NED: [0.04, -0.00] | Hdg: 303°
```

The EKF2 is now fusing AprilTag position with optical flow. Drift correction is active.

*Step 5: Monitor success* (SSH terminal 2)

Watch the EKF2 estimator status to confirm vision fusion:
```bash
sudo bash -c 'source /home/dexi/ros2_jazzy/install/setup.bash && \
  source /home/dexi/dexi_ws/install/setup.bash && \
  ros2 topic echo /fmu/out/vehicle_local_position --field xy_valid --field v_xy_valid --field heading'
```

Check that `/fmu/in/vehicle_visual_odometry` is being received:
```bash
sudo bash -c 'source /home/dexi/ros2_jazzy/install/setup.bash && \
  source /home/dexi/dexi_ws/install/setup.bash && \
  ros2 topic hz /fmu/in/vehicle_visual_odometry'
```

Should show ~10Hz when a tag is visible, 0Hz when no tag.

**What to look for:**
- `Origin locked` log confirms first tag detection and frame alignment
- Steady `Body:` / `NED:` values during hover = good position hold
- `Tag lost` log when flying away from tag — EKF2 falls back to optical flow
- If the drone holds position better over the tag vs. away from it, vision fusion is working

**What can go wrong:**
- `Waiting for heading/position/attitude...` — FC not connected or XRCE-DDS not running
- No `Tag detected` log — tag not in camera FOV, or `apriltag_ros` not running
- Drone oscillates over tag — try increasing `position_variance` or decreasing `publish_rate`
- Large NED values — normal, these are absolute EKF2 frame coordinates not relative to tag

### precision_landing

Autonomous precision landing on an AprilTag. Designed for body-fixed cameras with slow movement and filtering to prevent orbiting.

States: SEARCHING → DETECTED (5s wait) → CENTERING (1.5s stable lock) → LANDING → LANDED

LED feedback: Off → Purple → White → Red → Green

Features:
- Slow centering (10cm/s) to minimize camera tilt
- Moving average filter (5 samples) for smooth motion commands
- Uses raw position for threshold checks (prevents false "centered" during orbiting)
- Faster centering during descent (20cm/s) to track perspective changes
- Stable lock required before descent (must stay centered 1.5s)
- Safety abort: returns to CENTERING if offset exceeds 2x threshold during descent
- Pauses `px4_offboard_manager` setpoints during landing

Parameters:
| Parameter | Default | Description |
|-----------|---------|-------------|
| `centering_threshold` | 0.25m | Distance from center to consider "centered" |
| `centering_speed` | 0.10 m/s | Horizontal movement speed during CENTERING |
| `landing_centering_speed` | 0.20 m/s | Faster speed during LANDING (perspective changes) |
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
| `dry_run` | false | Log only, don't publish to EKF2 |
| `position_variance` | [2.0, 2.0, 100.0] | Position variance for EKF2 (XY trusted, Z ignored) |

## Prerequisites

- `apriltag_ros` running and detecting tags (included in `dexi_bringup`)
- PX4 flight controller connected via micro-ROS/XRCE-DDS (provides `/fmu/out/` topics)
- AprilTag 36h11 tags (any ID from 0-586)

## Dependencies

- `apriltag_ros` - AprilTag detection
- `px4_msgs` - PX4 message types
- `tf2_ros` - Transform lookups
- `cv_bridge` - Image conversion (visualizer node only)
