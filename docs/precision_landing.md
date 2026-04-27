# Precision Landing

Autonomous precision landing on a single AprilTag, validated end-to-end on the
**ARK Pi6X (PX4 v1.16.1) + CM4 + Pi Camera v2.1** stack with a 150 mm tag.

The script handles the full sequence:

1. **SEARCH** for the tag (LED off)
2. **DETECT** tag for a settle window (LED purple)
3. **CENTER** over the tag and hold (LED white) — auto-engages OFFBOARD mode
4. **LAND** with continuous re-centering (LED red)
5. **DISARM** on ground contact (LED green)

You take off manually in POSCTL on RC sticks. The script takes over when it
sees the tag in flight.

## What it does that's load-bearing

- **Locks the *tag's* NED position**, not the drone's, on first centered. The
  body-frame tag offset is rotated by drone heading and added to drone NED to
  get the tag's true world position. Continuously low-pass refined on each
  detection so the lock self-corrects after overshoot.
- **Locks target altitude** at CENTERING entry. Without this, re-reading live
  `drone_z` every loop creates a positive-feedback ratchet that walks the
  drone upward at full throttle-down.
- **Hysteresis** on the centering threshold (enter at 0.25 m, exit at 0.40 m)
  plus a 2-second tag-loss grace period — keeps the stable-lock timer from
  resetting on small fluctuations or brief tag flicker.
- **Airborne gate** (`min_takeoff_altitude`, default 0.30 m). The state
  machine refuses to engage while the drone is on the ground, even if the
  tag is in the camera frame. You can start the script before takeoff safely.
- **Auto-OFFBOARD switch** via `VEHICLE_CMD_DO_SET_MODE` at the
  DETECTED → CENTERING transition. No RC mode-switch or QGC click required.
- **Two-channel ground detection.** Subscribes to PX4's
  `/fmu/out/vehicle_land_detected` (uses thrust/IMU, fires reliably) AND keeps
  the original `altitude < 0.15 m AND vz ~ 0` check. The PAW3902 range sensor
  has ~0.5 m minimum detection range, so EKF altitude can stick above
  touchdown — `vehicle_land_detected` is the safety net.
- **Pauses `dexi_offboard_manager`** at 10 Hz via `/dexi/pause_setpoints`.
  Stops it from publishing trajectory setpoints in parallel with this script
  (which would cause jitter while the drone is in POSCTL).

## How to use

### One-time PX4 parameter setup

These need to be set on the FC for indoor flow + tag flight (set them via
`droneblocks-mavlink-tool/params.py` or QGC):

| Parameter | Value | Why |
|---|---|---|
| `EKF2_EV_CTRL` | **15** | All vision fusion bits — HPOS + VPOS + VEL + YAW. Without bit 3 (yaw), `yaw_align` never goes true and position fusion silently fails. |
| `EKF2_HGT_REF` | **2** | Range sensor as primary height — gives `acc_v` ~3 cm vs 30+ cm with baro. |
| `EKF2_RNG_CTRL` | **2** | Always fuse range sensor. |
| `EKF2_OF_CTRL` | **1** | Enable optical flow fusion. |
| `EKF2_OF_QMIN` | **50** | Lower flow quality threshold for indoor lighting. |
| `EKF2_GPS_CTRL` | **0** | Disable GPS (indoor). |
| `COM_ARM_WO_GPS` | **1** | Allow arming without GPS. |

### Tag setup

- Print AprilTag 36h11 ID 0 (or any matching the `target_tag_id` parameter).
- 150 mm black-square edge is what's been validated. 100 mm (4 in) tags
  flicker too often at hover altitudes above ~1 m.
- Tape flat to the floor, no glare, no curling.
- The tag size in the apriltag launch must match the print (default
  `'size': 0.15` in `dexi_bringup_*.launch.py`).

### Running the script

The drone bringup (`dexi.service`) must be running. Then on the drone:

```bash
ros2 run dexi_apriltag precision_landing.py \
  --ros-args -p stable_centering_duration:=60.0 -p detection_delay:=2.0
```

Common parameters:

| Parameter | Default | Notes |
|---|---|---|
| `target_tag_id` | 0 | Which AprilTag to land on |
| `detection_delay` | 5.0 s | Settle time after first detection before centering |
| `centering_threshold` | 0.25 m | Body-frame distance to declare "centered" |
| `centering_speed` | 0.10 m/s | Approach speed during CENTERING |
| `stable_centering_duration` | 1.5 s | How long the drone must hold centered before descent. Set higher (30–60 s) for a long hover demo before landing. |
| `descent_rate` | 0.3 m/s | Vertical speed during LANDING |
| `final_descent_rate` | 0.15 m/s | Speed below 0.5 m altitude |
| `landing_altitude` | 0.15 m | Altitude threshold for one of the disarm triggers |
| `min_takeoff_altitude` | 0.30 m | Airborne gate — won't engage until above this |

### Flight workflow

1. Start the script (it's safe to run with the drone on the ground — the
   airborne gate prevents engagement until you're flying).
2. Take off in POSCTL on RC sticks.
3. Fly to roughly over the tag at ~0.7 m altitude.
4. Hover steady when the tag is in the camera frame. Watch the LED:
   - **Purple** — tag detected, settling
   - **White** — script took over, OFFBOARD engaged. **Hands off the sticks.**
   - **Red** — descending
   - **Green** — landed and disarmed
5. RC mode-switch override always wins — flip back to POSCTL/ALTCTL/STAB to
   abort at any time.

## Tested results

Validated on Drone .57 (CM4 + ARK Pi6X + Pi Cam v2.1) on 2026-04-27:

- **Position discipline during 60-second hold:** within 6–12 cm of tag center
- **Altitude hold during 60-second lock:** rock steady, no drift-ratchet
- **Vision uptime:** ~91 % at hover altitudes around 0.7–1.5 m with the 150 mm tag
- **End-to-end auto-disarm:** ground contact triggered the altitude+vz path
  cleanly; the `vehicle_land_detected` safety net was not needed in the
  validation flight but is in place for the case where the range sensor
  sticks above touchdown.

## Known limitations / future work

- **Tag flicker.** The Pi Cam v2.1 has ~62° HFOV. At ~1 m altitude with the
  150 mm tag, lateral drift over ~30 cm pushes the tag out of the camera
  frame briefly. The hysteresis + grace-period absorb most of it.
- **Range sensor minimum range.** PAW3902 can't read below ~0.5 m. Below that,
  EKF altitude can stick at ~0.6 m even when the drone is on the ground.
  The `vehicle_land_detected` subscription handles this case via PX4's
  thrust/IMU-based detector.
- **Native PX4 precision land** (alternative future approach). On PX4 v1.16.1
  the path is to enter `NAVIGATION_STATE_AUTO_PRECLAND` (sub-mode **9**, NOT
  the regular `AUTO.LAND` sub-mode 6) — that's the mode wired to
  `LandingTargetEstimator`. Would also need `LTEST_MODE=1` (stationary
  target) and a node publishing `LandingTargetPose` to
  `/fmu/in/landing_target_pose`. The newer `LNDMC_LAND_PREC` opportunistic
  mechanism is post-v1.16 and not available on this build.
