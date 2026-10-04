# DEXI navigation framework

One stack, four layers, one estimator switch. Flow-only flight and AprilTag
fusion are the same code with a different PX4 profile; UWB slots into the same
place later. Developers, students, DroneBlocks blocks, Python and Node-RED all
drive the aircraft through the same command list.

```
 Layer 4  Interface      DroneBlocks blocks · Python · Node-RED
                         one service contract: ExecuteBlocklyCommand
 ──────────────────────────────────────────────────────────────────────
 Layer 3  Perception     tag_nav (dexi_apriltag)
          primitives     wait_for_tag · center_on_tag · fly_until_tag · land_on_tag · go_to_tag
 ──────────────────────────────────────────────────────────────────────
 Layer 2  Control        px4_offboard_manager (dexi_offboard)  — the ONLY setpoint owner
                         body-relative · velocity · absolute (goto_ned, hold_ned)
 ──────────────────────────────────────────────────────────────────────
 Layer 1  Estimation     PX4 EKF2
          flow profile:  optical flow + rangefinder          (relative NED, drifts)
          fusion profile: + apriltag_odometry → external vision (absolute in the tag map)
          later:          + UWB → external vision or GPS-like fixes
```

## Layer 1: estimation

Two PX4 profiles, switched by `EKF2_EV_CTRL` (0 = flow only, 15 = fuse external
vision) through the configurator's profile mechanism. Flow and range stay on in
both; tags only add absolute position and heading while a mapped tag is in view.

| Piece | Where | Status |
|---|---|---|
| `apriltag_node` detector, tag TFs | `dexi_bringup` | in bringup; CM4 feed raised 2 → 10 Hz |
| `apriltag_odometry` tag poses + map → external vision | `dexi_apriltag/src/apriltag_odometry.cpp` | exists (PR #3); needs the 0.105 m mount offset and the shared tag map |
| Profiles | `px4-web-configurator` | exist; need a switch reachable from the GCS |
| Tag map | one YAML shared by odometry and `tag_nav` | to do (today each would carry its own) |

A new positioning system is a new Layer 1 source. Nothing above changes. PX4
takes one external-vision source at a time, so tags plus UWB together means a
small combiner or UWB arriving as GPS-like fixes, which PX4 fuses alongside vision.

## Layer 2: control

`px4_offboard_manager` owns the 20 Hz setpoint stream, the offboard heartbeat
and the hand-back to the pilot. Nothing else publishes `/fmu/in/trajectory_setpoint`.

| Family | Commands | Trustworthy on |
|---|---|---|
| body-relative | `fly_forward/backward/left/right/up/down`, `yaw_left/right` | flow and fusion |
| velocity | `set_velocity_body`, `stop_velocity` | flow and fusion |
| hand-off | `start_setpoint_stream` (stream only; follows the aircraft until Offboard is entered, then latches) | flow and fusion |
| absolute | `goto_ned`, `hold_ned` (to add) | fusion; briefly on flow |

Two additions this framework needs from the manager:

- `hold_ned`: a pure position setpoint. `goto_ned` declares arrival at 0.25 m and
  re-latches the hold point at the current position, so no outer loop can close
  the last 20 cm through it (measured 2026-10-01: five centerings, all 19–24 cm).
- a status topic: control mode, target, whether a target is active, heartbeat
  on/off, setpoints paused. Today Layer 3 and the GCS are blind to it.

Also noted: velocity mode has no altitude hold of its own (Layer 3 compensates),
and the manager's internals are a candidate for PX4's ROS 2 Interface Library
(register as a real flight mode) once `hold_ned` and status exist.

## Layer 3: perception primitives

`tag_nav` (`dexi_apriltag/scripts/tag_nav.py`) reads the detector's TF directly and
drives Layer 2 by velocity or hold. It never publishes a setpoint. Service
`/dexi/tag_nav/execute`, same request type as the manager; status on
`/dexi/tag_nav/status` (JSON, 5 Hz).

| Command | What it does | Status |
|---|---|---|
| `wait_for_offboard` | the pilot hand-off: returns when armed, airborne, in OFFBOARD and `/dexi/tag_nav/engage` is true; engage comes from an RC aux switch (`engage_aux_index`), a Node-RED button (the `DEXI Tag Navigation` flow in node-red-dexi (`flows/tag_navigation.json`, shipped in the DEXI Node-RED image)), a block or a script; clearing it stands a running primitive down; it auto-clears on disarm | built, sim |
| `wait_for_tag` | returns when the tag is seen twice in a row | flown |
| `center_on_tag` (`-1` = whichever tag is in view) | tapered velocity chase to 0.25 m, then PX4 position hold at the tag's measured position, refined per detection; done inside 10 cm for 0.7 s | flown; needs `hold_ned` to reach the gate |
| `fly_until_tag` | body-frame velocity until the next tag is seen | prototype in the GCS, to port |
| `land_on_tag` | center, descend holding center, hand off to PX4 land on centering error and speed | prototype in the GCS, to port |
| `go_to_tag` | map lookup → `goto_ned` → `center_on_tag` | fusion only, to do |

Offset math and chase law are from `apriltag-corridor-mission-code` (flown on the
DEXI 5 and DEXI 10). Mount values live in `config/tag_nav_<airframe>.yaml`; measure
them per airframe. Gates: refuses unless airborne and in OFFBOARD; leaving
OFFBOARD stands it down; a tag lost for 8 s fails the command.

## Layer 4: interface

One service contract (`docs/COMMAND_CONTRACT.md`). Each DroneBlocks block is one
command; the GCS calls the aircraft's `tag_nav` when it exists and falls back to
its browser prototype only in the simulator. Node-RED and Python call the same
two services. A failed block lands the aircraft and reports why.

Node-RED runs server-side on the Pi, so a flow is a mission that needs no laptop
in the loop. The reference flow (the `DEXI Tag Navigation` flow in node-red-dexi (`flows/tag_navigation.json`, shipped in the DEXI Node-RED image)) has
ONE generic command node, not a node per capability: a function fills the request
and picks the service from the command name, one `ros2-service-call` executes it,
and a switch on `success` feeds the result back to a mission function that holds
the steps as data. Hand-off, ENGAGE button and START injects all converge on the
same mission node. Verified in the simulator 2026-10-02: hand-off → center tag 0
(5 cm) → fly until the next tag → center (7 cm) → fly → center (8 cm) → land.

## Student progression

The palette order is the curriculum: Setup, Takeoff, Navigation, Land (flow and
range, body-relative) → April Tags (perception) → absolute moves once the fusion
profile is on. Every block behaves the same in the corridor sim and on the aircraft.

## Gaps, in order

1. `hold_ned` + tolerance parameter in the manager.
2. Manager status topic.
3. One tag-map YAML for `apriltag_odometry` and `tag_nav` (`config/tag_map_avr2026_*.yaml` added; odometry still reads `tag_map_ids/x/y` params, to be loaded from it).
3b. Mount offset in one place. Today `tag_nav` applies it node-side, because the bringup's `base_link -> camera` transform is pitch-only and not a true optical-to-body rotation, so a body-forward translation written into it lands on the wrong axis. The right fix is a correct FRD transform in bringup, after which every consumer (`apriltag_odometry`, `tag_hop`, `precision_landing`, `tag_nav`) gets the mount for free and the node-side offsets go to zero. That is a frame change for all of them, so it gets its own retest.
4. `fly_until_tag`, `land_on_tag`, then `go_to_tag` in `tag_nav`.
5. Profile switch in the GCS.
6. Tag transforms in the SITL bringup (done 2026-10-01 in `dexi_bringup_unity_sim.launch.py`) so all of the above runs in the corridor sim first.
7. Yaw to the tag during the hold (tag yaw from the camera-to-tag rotation, slewed, locked inside 0.4 m).

## Measured on a DEXI 5 v1 (ARK Pi6X + CM4)

- Mount: lens 105 mm ahead of the frame center; bench check with the tag centered read 3 cm; pitch is a true 90°.
- Detector 2 Hz → 9 Hz with the dedicated throttle; velocity commands under ~0.1 m/s do not move the aircraft; a 0.15 m/s floor limit-cycles ±0.2 m against ~1.5 s of lag.
- `dist_bottom_valid` is false in flight with a sane reading: the EKF is not fusing the rangefinder, so its height is baro and read 0.4 m low. The manager's zero-velocity hold climbed 1–3 cm/s on it.
- Chase converges from 0.3–0.4 m in 2–4 s. Hold through `goto_ned` parks 19–24 cm off (the 0.25 m arrival tolerance).
