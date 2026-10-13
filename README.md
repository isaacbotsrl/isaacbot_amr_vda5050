<p align="center">
  <img src="icons/Logo-Arm-WhiteOrange-372x372-1.png" alt="Mechatronics Academy" width="140">
</p>

# isaacbot_amr_vda5050

VDA 5050 fleet interface for Ardennes: a master control sends orders over MQTT, and the isaacbot
drives them through `isaacbot_amr_mission_manager` and Nav 2. It runs as the `rover-a1-vda5050` container
of [`rover_docker`](https://github.com/RaduPotlog/rover_docker).

It is built on an existing implementation rather than written from scratch: InOrbit's
[`ros_amr_interop`](https://github.com/inorbit-ai/ros_amr_interop) connector (BSD-3), vendored
under `third_party/`. This repository adds only the isaacbot-specific adapter and the bringup.

**Protocol version: VDA 5050 2.0.0** (MQTT topics `uagv/v2/<manufacturer>/<serialNumber>/<topic>`).
No mature open-source robot-side connector implements 3.0 yet (surveyed 2026-09-28). A 3.0 master
control needs a later upgrade.

## Packages

| Package | What |
|---------|------|
| `third_party/ros_amr_interop/isaacbot_amr_vda5050_msgs` | VDA 5050 2.0 messages (upstream) |
| `third_party/ros_amr_interop/isaacbot_amr_vda5050_serializer` | ROS ↔ JSON (snake_case ↔ camelCase) (upstream) |
| `third_party/ros_amr_interop/isaacbot_amr_vda5050_connector` | `mqtt_bridge` (MQTT ↔ ROS), `controller` (order validation, base/horizon, stitching, action states, state/visualization/connection/factsheet) and the adapter node framework (pluginlib handlers) (upstream) |
| `isaacbot_amr_vda5050_adapter` | The isaacbot's handler plugins for that adapter, plus its `adapter_node` executable |
| `isaacbot_amr_vda5050_bringup` | `vda5050.launch.py`, `config/connector.yaml`, `config/mosquitto.conf`, `scripts/fake_master.py` |
| `isaacbot_amr_vda5050` | Metapackage |

`third_party/UPSTREAM.md` lists the upstream commit and every local patch: the lyrical port, the
paho-mqtt 2 API, `connection` retained with QoS 1, and forwarding `safetyState`.

## How it drives the isaacbot

```
master control ──MQTT── mosquitto ── mqtt_bridge ⇄ controller ──NavigateThroughNodes / ProcessVDAAction / GetState──► adapter
                                                                                                                         │ isaacbot_amr_vda5050_adapter plugins
                                             isaacbot_amr_mission_manager ◄── set_mission / run_mission / mission_state ───────┘
                                                      │ navigate_to_pose, one waypoint at a time
                                                    Nav 2
```

Orders go through `isaacbot_amr_mission_manager` rather than straight to Nav 2. That way every guard
the drive UI's GoTo has applies to them too: missions run only in the **Automatic** driving mode,
hold on the motion lock or a dead lidar, and abort on low battery. Nothing else sends Nav 2 goals
behind the manager's back.

| VDA 5050 | Isaacbot |
|----------|-------|
| Released base segment (`NavigateThroughNodes`) | One `set_mission` with the segment's nodes as waypoints, mission id `vda5050-<n>` |
| Order stitching (update extending the base) | `set_mission` with the nodes not yet reached plus the new ones (the manager replaces the mission in place) |
| Node reached | `mission_state.current_index` advancing |
| `cancelOrder` | `run_mission false` |
| `startPause` / `stopPause` | `run_mission false` and keep the route / `set_mission` with the nodes not yet reached (the manager has no pause) |
| `enableAuxOutput` / `disableAuxOutput` (instant, node) | `hardware_interface/aux_output_<n-1>/set` for each output named by `outputs` (see below) |
| `setDriveMode` (custom, instant) | `set_drive_mode` (`isaacbot_amr_msgs/SetDriveMode`) with `mode` `MANUAL` or `AUTOMATIC` (see below) |
| `startFollowing` / `stopFollowing` (custom, instant) | `follow_me/start` / `follow_me/stop` (`std_srvs/Trigger`, [isaacbot_amr_follow_me](https://github.com/isaacbotsrl/isaacbot_amr_orchestrator/tree/master/isaacbot_amr_follow_me), in `isaacbot_amr_orchestrator`) (see below) |
| `stateRequest`, `factsheetRequest` | Answered by the upstream controller. The factsheet is published retained on `…/factsheet`, and its `agvActions` lists every action here |
| `operatingMode` | Drive mode AUTOMATIC → `AUTOMATIC`; ASSISTED and MANUAL → `MANUAL` (an operator drives; `SEMIAUTOMATIC` would mean master control's orders run); no drive-mode manager → `SERVICE` |
| `paused` | `startPause`, or the mission manager holding the mission (motion lock, dead lidar) |
| `agvPosition` | TF `<ns>/<map_frame>` → `<ns>/base_link`; `positionInitialized` false when missing or older than 2 s. `mapId` = the indoor map in use, else `isaacbot.map_id` |
| `velocity` | `odom` twist |
| `batteryState` | `battery/battery_status` (charge 0 when the BMS does not know) |
| `safetyState.eStop` | `MANUAL` while the e-stop button is pressed or the safety PLC latch is set, else `NONE` |
| `safetyState.fieldViolation` | The drive mode's collision monitor is in its stop zone |
| `errors` (all `WARNING`) | `missionRefused` (why the manager refused the last order), `motionLocked`, `localizationUnavailable`, `safetyLinkDown` |

Isaacbot errors are never `FATAL`: the upstream controller stops dispatching navigation while any
`FATAL` error is present. A failed or refused order still ends with the controller's own `FATAL`
`noRouteError`, and master control has to `cancelOrder` it before sending a new order.

### Aux outputs

`enableAuxOutput` and `disableAuxOutput` switch the six aux outputs on the safety PLC. These
are general-purpose outputs, not part of the safety chain. Output 1 is DIO00 and output 6 is
DIO05, numbered 1..6 as in the drive UI. The only parameter is `outputs`: a single number or a
list.

```json
{"actionType": "enableAuxOutput", "actionId": "…", "blockingType": "NONE",
 "actionParameters": [{"key": "outputs", "value": [1, 3]}]}
```

- **Success:** the action is `FINISHED` once the PLC has acknowledged every write.
  `resultDescription` then says what was switched, for example `Aux outputs 1, 3 enabled.`
- **Bad parameter:** a missing `outputs`, or a number outside 1..6, writes nothing and ends
  `FAILED`.
- **Failed write:** the other outputs are still written, and the action ends `FAILED`. The
  reason is in the controller's `ACTION_FAILED` error.
- **Several outputs in one step:** put them in one action's list. The upstream adapter rejects a
  second action of the same type while the first is still running. That includes two
  `enableAuxOutput` in one `instantActions` message, and two `NONE`/`SOFT` ones on the same node.
- **Blocking type on a node:** a `HARD` or `SOFT` aux action ends the drivable segment, so the
  isaacbot stops on the node until the action finishes. The IO write is one Modbus round-trip per
  output, so the stop is short. A `NONE` action runs as the isaacbot drives through the node.
- **Services:** set by `isaacbot.aux_output_service_prefix` and `isaacbot.aux_output_timeout`.

### Drive mode

`setDriveMode` lets master control switch the isaacbot to Manual or Automatic. VDA 5050 has no
command for `operatingMode`: the robot only reports it. So this is a custom instant action.
Open-RMF's dashboard uses it (isaacbot_amr_rmf, Isaacbot card).

```json
{"actionType": "setDriveMode", "actionId": "…", "blockingType": "HARD",
 "actionParameters": [{"key": "mode", "value": "MANUAL"}]}
```

- **What it does:** it calls `isaacbot_amr_drive_mode`'s `set_drive_mode`, exactly as the drive UI's mode
  buttons do. The drive mode manager stays in charge. The new mode reaches master control through
  `operatingMode`.
- **`mode`:** `MANUAL` or `AUTOMATIC`. Case and quotes don't matter. Assisted can only be chosen
  on the drive UI, where someone is watching the isaacbot.
- **Success:** `FINISHED` with `resultDescription` `Drive mode Manual.` (or Automatic).
- **Refused:** `FAILED` with the manager's reason, e.g. Automatic while `isaacbot_amr_mission_manager`
  is missing. The same happens when `set_drive_mode` is unavailable or doesn't answer within
  `isaacbot.service_response_timeout`.
- **Served in Manual too:** VDA 5050's operating-mode table says a robot in `MANUAL` processes no
  instant actions. This one is served anyway, because handing a Manual isaacbot back to Automatic is
  what it is for. The upstream controller doesn't gate instant actions on the operating mode.
- **Not a safety function:** neither are the drive modes (isaacbot_amr_ros `isaacbot_amr_arch/SAFETY_CHAIN.md`).
  The e-stop, the safety PLC and the lidar monitor are unaffected.
- **Service:** `isaacbot.drive_mode_service` (default `set_drive_mode`).

### Follow-me

`startFollowing` and `stopFollowing` let master control start and stop follow-me
([isaacbot_amr_follow_me](https://github.com/isaacbotsrl/isaacbot_amr_orchestrator/tree/master/isaacbot_amr_follow_me), in `isaacbot_amr_orchestrator`). They are custom instant actions
without parameters:

```json
{"actionType": "startFollowing", "actionId": "…", "blockingType": "HARD", "actionParameters": []}
```

- **What they do:** they call `follow_me/start` / `follow_me/stop` (`std_srvs/Trigger`), exactly as
  the drive UI's *Follow me* card does. Fleet control decides *that* the isaacbot follows; the isaacbot
  follows on its own, through Nav 2's Following server, outside any order.
- **Success:** `FINISHED` as soon as following has started (`Following started.`) or stopped.
  Following itself does not show in the order or in `actionStates` after that.
- **Refused:** `FAILED`, and the controller publishes the reason once as an `actionFailed` error
  (`errorDescription` e.g. `startFollowing refused: drive mode is ASSISTED; switch the isaacbot to
  Automatic`); the upstream controller keeps `resultDescription` for FINISHED only. Reasons: the
  isaacbot is not in Automatic, an order is running, nobody stands in front of the isaacbot, or the
  services are unavailable (follow-me not running) or don't answer in time.
- **An order wins:** an order sent while the isaacbot follows starts a mission, and follow_me stops
  following on its own; send `stopFollowing` first to make it explicit.
- **Services:** `isaacbot.follow_me_start_service` / `isaacbot.follow_me_stop_service` (defaults
  `follow_me/start`, `follow_me/stop`).
- **Try it:** `fake_master.py follow start` / `fake_master.py follow stop`.

### Decisions worth knowing

- **Node headings** (`isaacbot.orientation_mode`, default `path`): the isaacbot arrives at each node
  facing along the edge it drove in on, and the node's `theta` is ignored. A skid-steer isaacbot
  turns in place poorly, and the ROS message cannot tell an omitted `theta` from `0.0`: honouring
  it would spin the isaacbot to 0 rad at every node of a master control that leaves `theta` out. Set
  `node` to use each node's `theta`.
- **Frame:** node coordinates are taken to be in Nav 2's global frame (`isaacbot.map_frame`, the
  container derives it from the localization source). The mission manager refuses missions in
  any other frame, with a message that ends up in `missionRefused`.
- **Refused orders** do not stop a mission the isaacbot is already running for someone else (e.g.
  an operator's GoTo). Only a mission this connector dispatched, or one whose `set_mission` timed
  out, is cancelled.
- **Not supported:** node and edge actions other than the aux-output actions, `initPosition`,
  edge `maxSpeed` and trajectories, loads, and the `allowedDeviation*` tolerances (the mission
  manager uses Nav 2's goal checker). An unsupported action is forwarded to the adapter, which
  rejects it, and it shows up as `FAILED` in `actionStates`.
- **Upstream limits:** order validation and the first-node deviation check are stubs, so any
  well-formed order and any start node are accepted. A stitch arriving just after a segment
  finished is acknowledged but not driven.
- A stitch re-sends the node being driven to, so Nav 2 re-plans once.

## Running

In the container, `ISAACBOT_AMR_VDA5050_ENABLE=true` is all it takes (see rover_docker's README, "VDA
5050"). Outside it:

```bash
mosquitto -c $(ros2 pkg prefix isaacbot_amr_vda5050_bringup)/share/isaacbot_amr_vda5050_bringup/config/mosquitto.conf &
ros2 launch isaacbot_amr_vda5050_bringup vda5050.launch.py namespace:=isaacbot map_frame:=map
```

Launch arguments: `namespace`, `broker_host`, `broker_port`, `broker_username`, `broker_password`,
`manufacturer`, `serial_number`, `map_frame`, `use_sim_time`, `params_file`.

`isaacbot_amr_mission_manager` must run with the same frame (`localization_source` gps/slam/amcl/indoor
→ `map`, odom → `odom`), and the isaacbot must be in Automatic.

## Testing

```bash
colcon build --packages-up-to isaacbot_amr_vda5050 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select isaacbot_amr_vda5050_adapter && colcon test-result --verbose
```

Unit tests cover the route tracking (progress, stitching, pause/resume, other missions' states),
the command use case (refusals, timeouts, cancel), the state mapping, and the aux-output
parameter parsing and use case. A domain-purity check
fails the build if `domain/` includes a ROS header.

`scripts/fake_master.py` is a minimal master control for manual tests (needs `python3-paho-mqtt`):

```bash
ros2 run isaacbot_amr_vda5050_bringup fake_master.py watch                  # print connection / state
ros2 run isaacbot_amr_vda5050_bringup fake_master.py order 2,0 4,0 4,2      # from the current position
ros2 run isaacbot_amr_vda5050_bringup fake_master.py order --order-id <id> --update 1 --from-node n4 6,2  # stitch
ros2 run isaacbot_amr_vda5050_bringup fake_master.py pause | resume | cancel | factsheet
ros2 run isaacbot_amr_vda5050_bringup fake_master.py aux on 1 3 | aux off 3    # aux outputs
ros2 run isaacbot_amr_vda5050_bringup fake_master.py drive-mode MANUAL | drive-mode AUTOMATIC
ros2 run isaacbot_amr_vda5050_bringup fake_master.py order 2,0 4,0 \
    --node-action 1:enableAuxOutput:2:HARD --node-action 2:disableAuxOutput:2   # on nodes
```

Gazebo has no aux IO, because `gz_ros2_control` replaces the isaacbot's hardware interface.
`scripts/fake_aux_io.py` stands in for it. It serves the six services, publishes
`aux_io_state`, and logs every write. `-p fail_output:=<n>` makes one output's writes fail.

```bash
ros2 run isaacbot_amr_vda5050_bringup fake_aux_io.py --ros-args -r __ns:=/isaacbot
```

For a real master control, NVIDIA's
[Mission Dispatch](https://github.com/nvidia-isaac/isaac_mission_dispatch) and
[openTCS](https://github.com/openTCS/opentcs) both speak 2.0. The
[vda5050_visualizer](https://github.com/bekirbostanci/vda5050_visualizer) watches the traffic over
the broker's WebSockets port (9001).

Verified 2026-09-28 in Gazebo (`isaacbot_amr_gazebo`, Nav 2 with `localization_source:=slam`,
`isaacbot_amr_drive_mode`, `isaacbot_amr_mission_manager`):
- the refusal in Assisted mode (`MANUAL`, `missionRefused`);
- a four-node order around an obstacle;
- `startPause` / `stopPause` mid-route (the isaacbot stops, then continues from the next node),
  including a resume one second after the pause;
- `cancelOrder` mid-drive;
- in-flight stitching;
- aux-output actions against `fake_aux_io.py`. Instant: on, off, a string list, output 7 and a
  missing `outputs` both `FAILED`, and a missing service `FAILED` with the adapter still up.
  Node: a `NONE` action on a node mid-segment, a `HARD` one, and a `NONE` one on the last node,
  in one order. All three ran in order while the route was driven.

Stitching needs `isaacbot_amr_mission_manager` from isaacbot_amr_orchestrator 60056e5 or later: before it,
replacing a running mission failed its first waypoint. `fieldViolation` was checked with a
stand-in isaacbot.

A leg that Nav 2 cannot plan fails the order like any other navigation failure (`noRouteError`,
then `cancelOrder`). Not yet run on the isaacbot.
