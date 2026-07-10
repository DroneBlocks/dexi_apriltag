# dexi_apriltag

AprilTag corridor navigation and odometry for DEXI drones.

## Overview

Fly along a corridor of AprilTags using tag-based visual odometry fused into PX4's EKF2. The drone steps through centerline waypoints, with tag detections continuously correcting position drift. If tags are lost, the drone descends to widen the camera FOV and retreats toward the last known good position.

### Nodes

| Node | Purpose |
|------|---------|
| `apriltag_odometry.py` | Publishes tag poses as VehicleOdometry to PX4 EKF2 |
| `corridor_navigation.py` | Flies centerline waypoints along a tag corridor (NED-based) |
| `tag_hop.py` | Body-frame velocity navigation through a tag sequence — see [tag_hop.py section](#tag_hoppy) |
| `precision_landing.py` | Autonomous precision landing on a specific tag — see [docs/precision_landing.md](docs/precision_landing.md) |
| `apriltag_visualizer` | Debug visualization of tag detections (C++) |

### How it works

1. `apriltag_ros` (from `dexi_bringup`) detects tags and publishes TF transforms
2. `apriltag_odometry.py` looks up tag TFs, maps them to NED positions using the tag map, and publishes `VehicleOdometry` to the EKF2
3. `corridor_navigation.py` sends `goto_ned` commands to the offboard manager, stepping through centerline waypoints
4. The EKF2 fuses tag position with optical flow velocity and range sensor altitude

## Simulation Setup (PX4 SITL + Unity)

### Prerequisites

- `dexi-sim-ftw` Docker stack running (PX4 SITL, ros2-dev, Unity sim)
- Unity `AprilTagNavigationScene` open
- `apriltag_ros` running in the ros2-dev container (started by default)

### Step 1: Set PX4 parameters

These don't persist across SITL restarts:

```bash
docker exec dexi-sim-ftw-px4-sitl-1 /opt/px4/bin/px4-param set EKF2_EV_CTRL 15
docker exec dexi-sim-ftw-px4-sitl-1 /opt/px4/bin/px4-param set MPC_XY_VEL_MAX 0.5
docker exec dexi-sim-ftw-px4-sitl-1 /opt/px4/bin/px4-param set MPC_XY_CRUISE 0.25
```

### Step 2: Start the odometry pipeline

In the ros2-dev container:

```bash
docker exec -it dexi-sim-ftw-ros2-dev-1 bash
source /opt/ros/jazzy/setup.bash && source /home/ubuntu/dexi_ws/install/setup.bash

ros2 launch dexi_apriltag apriltag_tag_map_sim.launch.py
```

This starts the static TF publisher (base_link → camera) and the odometry node with the 20-tag sim map.

### Step 3: Verify EKF2 fusion

```bash
ros2 topic echo /fmu/out/estimator_status_flags --field cs_ev_pos --once
# Should show: True
```

### Step 4: Run corridor navigation

```bash
ros2 run dexi_apriltag corridor_navigation.py --ros-args \
  -p flight_altitude:=2.0 \
  -p corridor_length:=9.0 \
  -p waypoint_spacing:=1.0
```

The drone will take off, wait for tag lock, then step through 10 centerline waypoints from NED(0,0) to NED(9,0), then land.

### Sim tag map

Defined in `config/tag_map_sim.yaml`. 2x10 grid of tag36h11 tags:
- 2 columns at East = -0.25m and +0.25m
- 10 rows from North = 0m to 9m (1m spacing)
- Tag size: 0.15m in Unity, 0.10m on real hardware

## Real Hardware Setup (ARK CM4)

### Prerequisites

- DEXI with ARK Pi6X flight controller
- Raspberry Pi CM4 companion computer
- Downward-facing CSI camera (IMX219)
- Optical flow + range sensor
- Printed AprilTag 36h11 tags (4-inch / 0.10m)

### Step 1: PX4 parameters (one-time via QGroundControl)

```
EKF2_EV_CTRL      1       # Vision horizontal position only
EKF2_HGT_REF      2       # Range sensor as primary height
EKF2_RNG_CTRL     2       # Always fuse range sensor
EKF2_OF_CTRL      1       # Enable optical flow
EKF2_OF_QMIN      50      # Lower flow quality threshold (indoors)
EKF2_GPS_CTRL     0       # Disable GPS
COM_ARM_WO_GPS    1       # Allow arming without GPS
```

Or use the param tool:
```bash
cd ~/droneblocks-mavlink-tool
source venv/bin/activate
python params.py 192.168.68.56 EKF2_EV_CTRL 1
```

### Step 2: Configure apriltag_ros with tag IDs

The `apriltag_node` in `dexi_bringup` must have `tag.ids`, `tag.sizes`, and `tag.frames` configured, otherwise it detects tags in 2D but doesn't publish TF poses. In the launch file:

```python
parameters=[{
    'image_transport': 'compressed',
    'family': '36h11',
    'size': 0.1,
    'tag.ids': [0, 1, 2, 3, ...],
    'tag.sizes': [0.1, 0.1, 0.1, 0.1, ...],
    'tag.frames': ['tag36h11:0', 'tag36h11:1', 'tag36h11:2', 'tag36h11:3', ...],
}]
```

### Step 3: Camera resolution

Set camera to 640x480 in `~/.dexi-config.yaml` (or `dexi_bringup/config/dexi_config.yaml`):

```yaml
nodes:
  camera:
    width: 640
    height: 480
```

At 320x240, libcamera crops instead of scaling, which halves the FOV and makes tag detection unreliable.

### Step 4: Create a tag map YAML

Measure your physical tag layout and create a YAML. Example for a 2x10 corridor:

```yaml
# config/tag_map_hw.yaml
tag_map:
  ids: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
  x:   [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 5.0, 5.0, 6.0, 6.0, 7.0, 7.0, 8.0, 8.0, 9.0, 9.0]
  y:   [-0.25, 0.25, -0.25, 0.25, -0.25, 0.25, -0.25, 0.25, -0.25, 0.25, -0.25, 0.25, -0.25, 0.25, -0.25, 0.25, -0.25, 0.25, -0.25, 0.25]
```

- **x**: NED North position (meters)
- **y**: NED East position (meters)

### Step 5: Start odometry and navigate

```bash
# Terminal 1: Start the odometry pipeline
sudo bash /home/dexi/start_apriltag_odom.sh

# Terminal 2: Run corridor navigation
sudo bash -c 'source /home/dexi/ros2_jazzy/install/setup.bash && \
  source /home/dexi/dexi_ws/install/setup.bash && \
  ros2 run dexi_apriltag corridor_navigation.py --ros-args \
    -p flight_altitude:=1.5 \
    -p corridor_length:=9.0 \
    -p waypoint_spacing:=1.0'
```

### SITL vs Hardware parameter differences

| Parameter | SITL | Hardware | Why |
|-----------|------|----------|-----|
| `EKF2_EV_CTRL` | **15** | **1** | SITL has no flow/range — tags provide everything. Hardware: tags only provide XY |
| `EKF2_HGT_REF` | default | **2** (Range) | Hardware uses real range sensor |
| `EKF2_RNG_CTRL` | default | **2** (Always) | Always fuse range sensor on hardware |
| `EKF2_OF_CTRL` | default | **1** | Enable real optical flow on hardware |

### How the three sensors complement each other

| Source | Provides | Rate | Weakness |
|--------|----------|------|----------|
| **Optical flow** | Velocity | ~50Hz | Drifts over time, no absolute position |
| **Range sensor** | Altitude | ~50Hz | Z axis only |
| **AprilTag odometry** | Absolute XY position | ~10Hz | Only works when tags visible |

## corridor_navigation.py

Flies along a tag corridor by stepping through centerline waypoints. Each waypoint pulls the drone back to the corridor center, preventing lateral drift.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `flight_altitude` | 1.5 | Cruising altitude (m) |
| `waypoint_spacing` | 1.0 | Distance between waypoints (m) |
| `corridor_length` | 9.0 | Total corridor length (m) |
| `corridor_east` | 0.0 | East coordinate of centerline |
| `position_threshold` | 0.3 | Arrival distance to advance (m) |
| `tag_lost_timeout` | 3.0 | Seconds without tags before recovery |
| `descend_step` | 0.3 | Meters to descend during recovery |
| `retreat_step` | 0.5 | Meters to retreat during recovery |
| `min_altitude` | 0.8 | Minimum altitude during recovery |

### Tag loss recovery

If tags are lost for longer than `tag_lost_timeout`:

1. **Stop** — halt forward progress
2. **Descend** — lower altitude to widen camera FOV
3. **Retreat** — fly back toward last position where tags were visible
4. **Land** — if recovery fails, land safely

## apriltag_odometry.py

Publishes tag poses as `VehicleOdometry` to `/fmu/in/vehicle_visual_odometry` for EKF2 fusion.

### Modes

- **Single tag mode** — one tag as implicit origin (default)
- **Tag map mode** — multiple tags at known world positions, provides absolute position

### Features

- **Tag switching hysteresis** — sticks with current tag unless another is >30% closer, prevents rapid bouncing between tags
- **Moving average filter** — 5-sample filter smooths position estimates
- **Origin locking** — aligns tag/map frame with EKF2 frame on first detection

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tag_family` | tag36h11 | AprilTag family |
| `target_tag_id` | 0 | Tag ID for single tag mode |
| `publish_rate` | 10.0 | Odometry publish rate (Hz) |
| `position_variance` | [10.0, 10.0, 100.0] | XY variance for EKF2 (higher = gentler corrections) |
| `filter_length` | 5 | Moving average samples |
| `dry_run` | false | Log only, don't publish to EKF2 |
| `tag_map_ids` | [] | Tag IDs for tag map mode |
| `tag_map_x` | [] | NED North positions |
| `tag_map_y` | [] | NED East positions |

## precision_landing.py

Autonomous precision landing on a specific AprilTag.

States: SEARCHING → DETECTED (5s wait) → CENTERING (1.5s stable lock) → LANDING → LANDED

LED feedback: Off → Purple → White → Red → Green

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_tag_id` | 0 | Tag ID to land on |
| `centering_threshold` | 0.25 | Distance to consider centered (m) |
| `centering_speed` | 0.10 | Centering speed (m/s) |
| `descent_rate` | 0.3 | Descent speed (m/s) |
| `detection_delay` | 5.0 | Wait after first detection (s) |
| `stable_centering_duration` | 1.5 | Hold center before descent (s) |

```bash
ros2 launch dexi_apriltag precision_landing.launch.py target_tag_id:=18
```

## tag_hop.py

Body-frame velocity navigation through a sequence of AprilTag waypoints. Drone takes off in POSCTL, the script auto-engages OFFBOARD when it sees the first tag in the sequence airborne, then walks through the sequence: center on tag, hold, fly forward (or backward) until the next tag enters the camera FoV, center, hold, repeat. When the sequence completes, the script hands off to PX4 AUTO.LAND for descent and disarm.

### TL;DR

Lay your tags in a line. The default corridor is `0 → 2 → 4 → 2 → 0` with 2 m spacing along +N.

```bash
ros2 launch dexi_apriltag tag_hop.launch.py
```

To hover and hold on a single tag instead of walking a corridor, use `tag_hold.launch.py` (sequence `[0]`, no transit):

```bash
ros2 launch dexi_apriltag tag_hold.launch.py            # hold over tag 0
ros2 launch dexi_apriltag tag_hold.launch.py yaw_align:=true   # + hold heading North
```

This brings up everything `tag_hop` needs:
- `base_link → camera` static TF
- `apriltag_odometry` (vision-corrected EKF — recommended for stable holds)
- `tag_hop` itself

For non-default corridor layouts (different sequence or spacing), copy `launch/tag_hop.launch.py` and edit the `sequence` / `tag_map_ids` / `tag_map_n` / `tag_map_e` constants. ROS2 launch substitutions don't support array params, so they're hardcoded in the launch file rather than CLI args. Scalar params (`hover_duration`, `transit_speed`, etc.) ARE launch args:

```bash
ros2 launch dexi_apriltag tag_hop.launch.py hover_duration:=5.0 transit_speed:=0.15
```

Take off, fly steady over the first tag, **let go of the sticks** when the LED goes purple → cyan. The script flies the rest. RC mode flip aborts.

### LED reference

| LED | Phase |
|---|---|
| off | Searching for first tag (or aborted) |
| purple | Tag detected, 2 s settle |
| `detection_led_color` (default `cyan`) | Centering or holding on a tag |
| yellow | Body-frame transit to next tag |
| red | PX4 AUTO.LAND descending |
| green | Landed and disarmed by PX4 |

### Why body-frame velocity instead of NED waypoints

PX4's EKF NED drifts on flow + IMU (we measured 1+ m of drift on a stationary tag in a single short flight). NED-target setpoints fight a moving estimate — drone goes the wrong direction. Body-frame velocity uses gyro+accel attitude (very accurate) so the drone moves correctly in the real world regardless of EKF state. Vision (TF for the next sequence target) decides arrival, not EKF distance. EKF can be off by meters and the corridor still works.

### Workflow

1. Print AprilTags (`tag36h11`, IDs from `tags/` directory). 167 mm prints validated.
2. Configure `apriltag_node` in your bringup launch with `tag.ids`, `tag.sizes`, `tag.frames` for every tag in your sequence — without these, no TF is published.
3. Take off in POSCTL. Approach the first tag **slowly** and hold a steady hover above it. A "hot" entry leaves residual velocity that fights the centering chase at OFFBOARD handoff.
4. When LED goes **purple**, hands off the sticks. Cyan = script in control.
5. Watch the LEDs. Manual override (any RC mode flip) aborts; you can land manually and take off again to re-engage.

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `sequence` | `[0, 1, 0]` | Tag IDs to visit, in order |
| `tag_map_ids` / `tag_map_n` / `tag_map_e` | `[0, 1]` / `[0, 1]` / `[0, 0]` | Tag positions (meters, NED). Used only to decide forward vs backward direction during TRANSIT |
| `hover_duration` | 10.0 | Seconds to hold over each tag |
| `transit_speed` | 0.20 | Body-frame velocity during TRANSIT (m/s) |
| `centering_speed` | 0.20 | Body-frame velocity while chasing a tag in CENTERING (m/s) |
| `min_transit_duration` | 1.0 | Minimum seconds in TRANSIT before allowing tag acquisition (forces visible body-frame motion when adjacent tags overlap in FoV) |
| `transit_timeout` | 15.0 | Max seconds in TRANSIT before handing off to PX4 AUTO.LAND |
| `centering_threshold` | 0.25 | Body-frame distance to declare centered (m) |
| `tag_loss_grace` | 2.0 | Seconds the tag can be lost before resetting CENTERING timers |
| `min_takeoff_altitude` | 0.30 | Airborne gate — won't engage until above this (m). Reads the downward range sensor (`dist_bottom`), not the EKF `z` estimate, which can drift or invert on flow-only flight |
| `detection_delay` | 2.0 | Settle window after first detection before engaging (s) |
| `detection_led_color` | `cyan` | LED color while centering or holding on a tag |
| `yaw_align` | `false` | Hold a fixed heading while centering/holding instead of free yaw. Use it when the drone slowly yaws off heading (no magnetometer, flow-only) |
| `yaw_align_deg` | 0.0 | Heading to hold when `yaw_align` is on (0 = North) |

### Coexistence with `dexi_offboard_manager`

`tag_hop.py` publishes directly to `/fmu/in/trajectory_setpoint` and `/fmu/in/offboard_control_mode`. It pauses `dexi_offboard_manager`'s setpoints via `/dexi/pause_setpoints`, but `dexi_offboard_manager`'s OffboardControlMode heartbeat keeps publishing — and its flags can conflict with what `tag_hop` sends, causing motion glitches.

**Workaround:** kill `px4_offboard_manager` before running `tag_hop`:
```bash
sudo pkill -9 -f px4_offboard_manager
```
A future PR will refactor `tag_hop` to call into `dexi_offboard_manager` rather than publish directly, eliminating this conflict.

### `apriltag_odometry` is NOT required

`tag_hop.py` does its own TF lookups against `apriltag_node` directly — it doesn't depend on `apriltag_odometry` feeding the EKF. Running `apriltag_odometry` alongside is fine for the holding phase (vision keeps EKF anchored, less drift), but if any tag in your sequence is in `apriltag_odometry`'s `tag_map_ids` and you fly over it mid-transit, the EKF gets a position correction that PX4 acts on — you'll feel a jerk. Either exclude waypoint tags from `apriltag_odometry`'s map, or kill `apriltag_odometry` entirely:
```bash
sudo pkill -9 -f apriltag_odometry
```

### Validated configurations

- **DEXI (CM4 + ARK Pi6X + Pi Cam v2.1)**, indoor, PX4 v1.16.1, EKF on optical flow + range sensor (no GPS)
- 167 mm printed tags
- Corridor 4 m long: tags 0, 2, 4 at 0/2/4 m N
- `transit_speed=0.20`, `centering_speed=0.20`, `min_transit_duration=1.0`
- Sequence `[0, 2, 4, 2, 0]` flown end-to-end with AUTO.LAND completion

- **DEXI-3 (CM5 + H743-AIO + Arducam fixed-focus IMX708)**, indoor, PX4 v1.17.0, optical flow + range sensor (no GPS)
- Single-tag hold (`tag_hold.launch.py`, 167 mm tag 0) engaged and held via the range-sensor airborne gate

## Printing tags

Tag PNGs are in `tags/` (tags 0-19, tag36h11 family, 360x360px at 72 DPI for 4-inch print). Print at exactly 4 inches (0.10m) — tag size must match the `size` parameter in `apriltag_ros`.

## Dependencies

- `apriltag_ros` — tag detection and TF publishing
- `px4_msgs` — PX4 message types
- `tf2_ros` — transform lookups
- `dexi_interfaces` — `OffboardNavCommand` messages
- `cv_bridge` / `OpenCV` — visualizer node only
