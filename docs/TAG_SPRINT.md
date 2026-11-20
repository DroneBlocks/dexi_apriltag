# Tag sprint: down a line of AprilTags and back

Take off on a tag, fly down a line of tags at 1.0 m/s, turn around on the far tag, fly back and
land on the tag you started from. Three tags, a tape measure and a switch flip. The script is
`examples/python/tag_sprint.py`.

```
 tag 1 (start, land) ---- 1.0 m/s ---->  tag 4 (checkpoint) -- 0.6 m/s -->  tag 5 (finish)
          ^                                                                     |
          +----------------------------- 1.0 m/s ------------------------------+
```

Flown on a DEXI-5 and a DEXI-3 in November 2026 (results at the end).

## What you need

- A DEXI running DEXI-OS with `tag_nav` enabled in bringup (`tag_nav:=true`), dexi_apriltag at
  or after the rate-hold fix (`hold_refine: rate`).
- A down camera and rangefinder. The tags must be in view at the flight height.
- Three tag36h11 tags, 6 to 7 in black square, printed at 100%, flat on the floor in a straight line.
  Defaults: start **1**, checkpoint **4**, finish **5**. Any IDs work with `--start/--checkpoint/--finish`.
- About 8 m of open floor plus 2 m past the finish tag, 2 m of ceiling.

## 1. Lay out and measure the tags

Put the start and finish tags 5 to 7 m apart with the checkpoint 1 to 2 m before the finish.
They do not need precise placement: the drone recenters on each tag. Measure center to center:

| | Default | Your course |
|---|---|---|
| start -> checkpoint | 4.93 m | `--checkpoint-m` |
| start -> finish | 6.41 m | `--finish-m` |

The distances only set the safety timeouts (1.3 x the leg + 0.7 m), so within 10 cm is plenty.

## 2. Radio and PX4

| What | Setting | Why |
|---|---|---|
| Mode switch | a Position slot (`COM_FLTMODEn = 2`) | arm and fly by hand in Position |
| Offboard switch | `RC_MAP_OFFB_SW` = the switch channel (we use CH7) | the flip starts the sprint and is the abort |
| Arming | sticks, or `RC_MAP_ARM_SW` if you arm from a switch | |
| Kill switch | `RC_MAP_KILL_SW` off before arming | |
| No compass fitted | `SYS_HAS_MAG = 0` | otherwise "Found 0 compass" blocks arming |

Optional, faster takeoff and landing (what the timed runs used): `MPC_TKO_RAMP_T 1.0`
(default 3.0) and `MPC_LAND_ALT3 0.3` (crawl only the last 30 cm, default 1.0).

## 3. Start the script on the drone

```bash
ssh dexi@<drone-ip>
sudo -E bash -c 'export ROS_DOMAIN_ID=<your domain>; \
  source ~/ros2_jazzy/install/setup.bash; source ~/dexi_ws/install/setup.bash; \
  python3 -u ~/dexi_ws/src/dexi_apriltag/examples/python/tag_sprint.py \
    --checkpoint-m 4.93 --finish-m 6.41'
```

Use the same `ROS_DOMAIN_ID` as the drone's bringup. Give every drone on the network its own.

The script sets the down camera to a fixed **4 ms exposure** (gain 10.6) when it starts. With
auto-exposure, frames blur above about 0.75 m/s and the drone flies past tags. A fixed exposure
does not follow the room light; if the shop gets darker over a long session, restart the script
in brighter light or raise `--gain`. `--exposure-us 0` leaves the camera alone.

Wait for `READY`.

## 4. Fly

1. Drone on the ground over the start tag, nose toward the finish tag. Off-center is fine.
2. Mode switch to **Position**, arm.
3. Flip the **Offboard** switch. It takes off to 1.5 m, centers, flies the sprint and lands.
4. **Abort:** flip the Offboard switch back at any time and fly it in Position.
5. Next run: Offboard switch back, carry it to the start tag, re-arm, flip again.

If a tag is not found or centering fails, the drone **holds** and the log says
`take over with the Offboard switch`.

## 5. Read the result

The last line of each run:

```
SPRINT ok at 1.00 m/s: 42.8 s from the switch to disarmed
```

plus the landing offset from the tag on the `landing on tag` line. Record a flight bag to
review it later (`ros2 bag record` of `/apriltag_detections /tf /dexi/tag_nav/status
/cam0/image_raw/compressed /fmu/out/vehicle_local_position`).

## Results so far

| Drone | Course | Speed | Time (switch to disarmed) | Landing offset |
|---|---|---|---|---|
| DEXI-3 | 1 -> 4 -> 5 -> 1, down and back | 1.0 m/s | 42.8 s | 4.5 cm |
| DEXI-5 | 1 -> 4 -> 5, one way | 1.25 m/s | 28.8 s | 5.7 cm |
| DEXI-5 | 1 -> 4 -> 5, one way, fast takeoff/land params | 1.75 m/s | 22.1 s | 2.9 cm |

What we learned:

- Takeoff, centering and landing take most of the time. Going from 1.25 to 1.75 m/s saved 1 s;
  the takeoff and landing parameters above saved 5.6 s.
- 2.0 m/s overshot the finish tag and lost more time recentering than it gained.
- The checkpoint is a full stop (tag_nav brakes when it sees each tag), which is what keeps the
  stop on the finish tag accurate.
- If the land detector does not confirm touchdown, the drone sits armed on the ground; the
  script forces a disarm after 6 s.
