# DEXI command contract

Every interface (DroneBlocks blocks, Python, Node-RED) drives the aircraft through
one request type and two services. Nothing else is needed to fly a mission.

```
dexi_interfaces/srv/ExecuteBlocklyCommand
  request:  command (string)  parameter (float)  timeout (s, 0 = none)
            north east down yaw (float)     index r g b (int, LEDs only)
  response: success (bool)  message (string)  execution_time (s)
```

| Service | Owner | Commands |
|---|---|---|
| `/dexi/execute_blockly_command` | `dexi_offboard` `px4_offboard_manager` | flight, body-relative moves, absolute NED, hand-off stream |
| `/dexi/tag_nav/execute` | `dexi_apriltag` `tag_nav` | perception primitives (AprilTags) |

Both services block until the command completes or `timeout` elapses, and answer
`success=false` with a reason instead of raising. A client (the flow, a block, a
script) sends the next command only after the previous one answered.

## `/dexi/execute_blockly_command`

| command | parameter | north / east / down / yaw | completes when |
|---|---|---|---|
| `arm` / `disarm` | | | PX4 confirms |
| `takeoff` | altitude m | | altitude reached (PX4 native takeoff) |
| `offboard_takeoff` | altitude m | | altitude reached in Offboard |
| `land` | | | landed and disarmed |
| `fly_forward` `fly_backward` `fly_left` `fly_right` `fly_up` `fly_down` | distance m | | within `position_tolerance` (0.25 m) of the body-relative target |
| `yaw_left` `yaw_right` | degrees | | within `heading_tolerance` |
| `goto_ned` | | target N E D (m), yaw (deg) | within tolerance, then the hold re-latches on arrival |
| `hold_ned` | | hold point N E D (m), yaw (deg) | immediately; the manager streams that exact point until the next command |
| `set_velocity_body` | | body velocity m/s (FRD), yaw = yaw rate deg/s | immediately; velocity runs until `stop_velocity` or another command |
| `stop_velocity` | | | immediately; position hold at the current pose |
| `circle` | radius m | | trajectory finished |
| `start_setpoint_stream` | | | immediately. Heartbeat **without** commanding Offboard: the setpoint follows the aircraft until the pilot enters Offboard, then latches where the aircraft is. The pilot hand-off uses this. |
| `start_offboard_heartbeat` | | | immediately. Heartbeat **and** commands Offboard 1 s later (GCS missions from the ground). |
| `stop_offboard_heartbeat` | | | immediately |
| `switch_offboard_mode` / `switch_hold_mode` | | | immediately |

Leaving Offboard: with `start_offboard_heartbeat` the stream stops (the pilot has
the aircraft). With `start_setpoint_stream` the stream keeps running and follows
the aircraft again, so the pilot can re-enter Offboard later without a reset.

## `/dexi/tag_nav/execute`

| command | parameter | north / east / down | completes when |
|---|---|---|---|
| `center_on_tag` | tag id, **-1 = any tag in view** | | held inside 0.25 m of the tag for `hold_accept_s` (8 s), typically 5–10 cm; fails after 8 s without the tag |
| `fly_until_tag` | tag id, **-1 = the first tag that was not in view at the start** | body velocity m/s (FRD), e.g. north 0.25 | the tag is seen on 2 consecutive detections after `transit_min_s`; the aircraft is left in a position hold |
| `wait_for_tag` | tag id | | the tag is in view |
| `wait_for_offboard` | | | armed, airborne, in OFFBOARD and engaged (`/dexi/tag_nav/engage`, RC aux, or Offboard entry when `engage_on_offboard`) |

Gates on every `tag_nav` command: airborne (rangefinder, or EKF height while
armed) and in OFFBOARD. Leaving OFFBOARD or clearing engage stands the node down
and fails the running command. Transit speed: **0.25 m/s**; 0.4 m/s outruns the
detector on a 6 in tag at 1.25 m.

Status for dashboards and flow logic: `/dexi/tag_nav/status` (std_msgs/String,
JSON, 5 Hz: `state`, `tag`, `visible`, `offset`, `alt`, `engaged`, `armed`,
`nav_state`) and `/dexi/offboard_manager/status` (JSON, 5 Hz: `control_mode`,
`target`, `heartbeat`, `armed`, `landed`).

## Node-RED

the `DEXI Tag Navigation` flow in node-red-dexi (`flows/tag_navigation.json`, shipped in the DEXI Node-RED image) is the reference flow. One generic
command node (a function that fills the request and picks the service by command
name, wired to one `ros2-service-call`) executes every command above; the mission
is a list of steps inside a single function node. Starting a mission: the RC
hand-off (`/dexi/tag_nav/status` rising edge of engaged + OFFBOARD + armed), the
ENGAGE button (publishes engage and sends `switch_offboard_mode`), or START for
bench and sim use. A failed step lands. Import into the aircraft's Node-RED
(port 1880); the flow uses the `ros2-websocket-server` node the DEXI image ships.
