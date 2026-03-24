# AprilTag Grid Setup Guide

How to set up a physical AprilTag grid for indoor precision navigation with DEXI.

## What's Included

This folder contains 20 AprilTag 36h11 PNGs (IDs 0-19) for printing.

## Printing

- Print each tag on a standard **8.5×11 inch** sheet of paper
- The PNGs are pre-sized to **540×540 pixels at 72 DPI = 7.5 inches total**
- At that size the **black area is 6 inches (15cm)** with a 0.75-inch white margin on each side
- Just print at **100% scale** (no fit-to-page) — the tag will be centered with the correct margins
- Use **matte paper** (not glossy — glare interferes with detection)
- Standard black and white laser or inkjet printing works fine
- Do **not** laminate (causes reflections)
- **Orientation doesn't matter** — tags can be placed at any rotation, the detector handles it

## Grid Layout

Lay out a 2-column × 10-row grid on the floor:

```
         0.5m
       ←─────→
       col A  col B

Row 1: [Tag 0] [Tag 1]   ← START (drone takes off here)
       ────── 1m ──────
Row 2: [Tag 2] [Tag 3]
       ────── 1m ──────
Row 3: [Tag 4] [Tag 5]
       ────── 1m ──────
Row 4: [Tag 6] [Tag 7]
       ────── 1m ──────
Row 5: [Tag 8] [Tag 9]
       ────── 1m ──────
Row 6: [Tag 10] [Tag 11]
       ────── 1m ──────
Row 7: [Tag 12] [Tag 13]
       ────── 1m ──────
Row 8: [Tag 14] [Tag 15]
       ────── 1m ──────
Row 9: [Tag 16] [Tag 17]
       ────── 1m ──────
Row 10:[Tag 18] [Tag 19]  ← END (9m from start)
```

**Total flight path: 9 meters (~30 feet)**

## Step-by-Step Floor Setup

1. **Mark the center line** with painter's tape — this is the flight path (9m long)

2. **Mark cross lines** every 1 meter along the center line (10 marks total)

3. **At each cross mark**, place two tags — one 25cm left of center, one 25cm right of center (0.5m apart)

4. **Tape tags flat to the floor** — ensure no curling at edges. Use tape on all four corners

5. **Tag IDs must match the positions** shown above. Tags 0 and 1 at the start, incrementing by 2 per row

## Center-to-Center Distances

| Measurement | Metric | Imperial |
|---|---|---|
| Same row (left ↔ right) | 0.5m | ~19.7 in |
| Adjacent rows (along flight path) | 1.0m | ~39.4 in |
| Diagonal (adjacent row, opposite column) | ~1.12m | ~44 in |

## What Matters

- **1m row spacing** is the most critical measurement — use a tape measure. The tag map config assumes exact positions
- **0.5m column spacing** is less critical — both tags just need to be in the camera's field of view
- **Tags must be flat** — any warping degrades pose estimation
- **Orientation doesn't matter** — tags can be rotated any direction, detection is rotation-invariant

## Camera Coverage

At 2m flight altitude with the Pi Camera v2 (62° HFOV):
- Camera sees a ~2.4m × 1.8m patch of floor
- **4 tags visible** at any point along the path
- If the drone drifts sideways, tags remain in view (field is wider than the grid)

## PX4 Parameters for Real Hardware

Set these on the flight controller (one-time via QGC MAVLink console):

```
param set EKF2_EV_CTRL 1        # Fuse vision XY position (tags handle XY only)
param set EKF2_HGT_REF 2        # Range sensor for height
param set EKF2_RNG_CTRL 2       # Always fuse range sensor
param set EKF2_OF_CTRL 1        # Enable optical flow
param set EKF2_OF_QMIN 50       # Lower flow quality threshold for indoors
param set EKF2_GPS_CTRL 0       # Disable GPS (indoor)
param set COM_ARM_WO_GPS 1      # Allow arming without GPS
param set MPC_XY_VEL_MAX 0.5    # Max horizontal velocity (m/s)
param set MPC_XY_CRUISE 0.25    # Cruise speed (m/s)
```

## Bringup Launch File

Update the tag size in your hardware launch file before flying:

```python
# In dexi_bringup_pi5.launch.py (or dexi_bringup_ark_cm4.launch.py):
'size': 0.15,  # Must match physical tag size (6 inches = 0.15m)
```

## Testing Before Flight

1. **Verify detections**: Hold the drone by hand ~1m above the tags and check:
   ```bash
   ros2 topic echo /tf
   ```
   You should see transforms for visible tags with Z distances ~1m

2. **Dry run odometry**: Start the odometry node in dry_run mode and move the drone by hand:
   ```bash
   ros2 launch dexi_apriltag apriltag_odometry.launch.py dry_run:=true
   ```
   The logged NED positions should track your hand movements

3. **Start small**: Lay out just 5m of tags (rows 1-5, tags 0-9) for initial testing. Extend to 9m once validated

## Tag Map Configuration

The tag map in `config/tag_map_sim.yaml` maps tag IDs to NED world coordinates. For this grid layout:

- **x** = NED North (meters along flight path): 0, 0, 1, 1, 2, 2, ... 9, 9
- **y** = NED East (meters across): -0.25, +0.25 (alternating for left/right columns)

If you change the physical spacing, update the tag map YAML to match.
