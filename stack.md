# Stack: what talks to what

Three layers, two buses. You only write the tracking law inside the ROS layer.

```text
YOU LOOK AT
  Gazebo window ............. physics, the rover moving
  RViz ...................... green = path file, red = driven
  QGroundControl (optional) . joystick / mode, not required for control.launch

ROS 2 (Jazzy) ............... nodes, topics, services
  your C++  →  MAVROS  →  (leaves ROS here)

NOT ROS
  MAVLink UDP ............... firmware protocol (MAVROS and QGC)
  JSON :9002 ................ Gazebo plugin ↔ SITL (sensors in, wheel cmds out)
```

**ROS 2** is how nodes in this repo talk.  
**MAVLink** is how MAVROS talks to ArduRover. You never pack MAVLink; MAVROS does.

If a diagram mixes “topic” and “UDP port”, it is mixing those two buses.

---

## 1. Processes (what launch actually starts)

`sim.launch.py` starts the left column. `control.launch.py` starts that plus the right column.

```text
                    sim.launch.py                         control.launch.py adds
                    ─────────────                         ─────────────────────
                    gz sim  (Gazebo)
                         │
                         │  plugin JSON :9002
                         ▼
                    ArduRover SITL
                    (sim_vehicle.py)
                         │
              ┌──────────┴──────────┐
              │ UDP 14551           │ UDP 14550
              ▼                     ▼
           MAVROS                 QGC (optional,
           (ROS node)              not launched)
              ▲
              │ ROS
              │
           ros_gz_bridge ──────────────────────────────► /ground_truth/odom
                                                            │
                                                            ├─► trajectory_controller_node
                                                            ├─► path_scorer_node
                                                            └─► RViz
```

Timing inside `sim.launch.py`: Gazebo immediately, bridge at 3 s, SITL at 5 s, MAVROS at 8 s (SITL must be listening on 14551 first).

---

## 2. ROS graph (names you can `echo`)

Everything here is a ROS topic or service. Arrows are publish → subscribe.

```text
Gazebo
  gz topic /ground_truth/odom
        │
        │ ros_gz_bridge
        │   /ground_truth/odom @ nav_msgs/Odometry  [  gz.msgs.Odometry
        ▼
/ground_truth/odom          nav_msgs/Odometry     (map / ENU, ground truth)
        │
        ├──────────────────────► trajectory_controller_node
        │                              │
        │                              │ 20 Hz timer: SetupArdurover() then Control()
        │                              │
        │                              ├─► /path_markers          MarkerArray
        │                              │         green target_path, red driven_path
        │                              │
        │                              ├─► /mavros/setpoint_velocity/cmd_vel
        │                              │         geometry_msgs/TwistStamped
        │                              │
        │                              ├─► service /mavros/set_mode      GUIDED
        │                              ├─► service /mavros/cmd/arming    true
        │                              └─► param   /mavros/setpoint_velocity.mav_frame
        │                                           BODY_NED
        │
        ├──────────────────────► path_scorer_node  →  paths/score.txt
        └──────────────────────► RViz

/mavros/state               mavros_msgs/State     connected, armed, mode
        └──────────────────────► trajectory_controller_node
```

RViz also subscribes to `/path_markers`. It never publishes a drive command.

`path_recorder_node` (only if you start it yourself with `sim.launch.py`) also subscribes to `/ground_truth/odom` and writes `paths/recorded.path`.

---

## 3. One control tick (closed loop)

This is the only loop that moves the robot. Read top → bottom, then back.

```text
  1. Gazebo has a pose for the Husky
           │
           ▼
  2. ros_gz_bridge publishes /ground_truth/odom
           │
           ▼
  3. Control(odom) computes a body velocity
           │
           │  TwistStamped
           │  twist.linear.x  = forward  (m/s)
           │  twist.angular.z = yaw rate (rad/s)
           ▼
  4. MAVROS  /mavros/setpoint_velocity/cmd_vel
           │
           │  MAVLink SET_POSITION_TARGET_LOCAL_NED
           │  UDP 127.0.0.1:14551
           ▼
  5. ArduRover SITL   must be armed + GUIDED, else ignore
           │
           │  PWM values for the 4 wheel channels (1000–2000 µs)
           │  stuffed into a JSON UDP packet  →  127.0.0.1:9002
           ▼
  6. Gazebo ArduPilotPlugin maps PWM → joint velocity (rad/s)
           │
           ▼
  7. Physics integrates  →  new pose  →  back to 1
```

If step 5 is MANUAL, 6 never happens from your Twist. Gazebo still runs; the rover sits.

---

## 4. Ports and protocols

SITL opens two MAVLink outlets and one physics socket. Different clients, same firmware.

| Link | Address | Protocol | Payload |
|---|---|---|---|
| Gazebo plugin ↔ SITL | `127.0.0.1:9002` | JSON (ArduPilot FDM) | IMU/pose in; wheel servo out |
| MAVROS ↔ SITL | `udp://:14551` (`--out 127.0.0.1:14551`) | MAVLink | setpoints, arm, mode, `/mavros/state` |
| QGC ↔ SITL | `127.0.0.1:14550` | MAVLink | RC override, mode, HUD |
| ros_gz_bridge ↔ Gazebo | in-process / gz transport | gz.msgs → ROS | `/ground_truth/odom` only |

QGC and MAVROS do **not** talk to each other. Both talk to SITL.

---

## 5. Components

| Piece | How it starts | What it is |
|---|---|---|
| **Gazebo** | `scripts/run-gz.sh` → `gz sim -r sim/worlds/baylands.sdf` | Physics + 3D. Husky model, IMU, odom publisher. No autopilot logic. |
| **ArduPilotPlugin** | loaded from `sim/models/clearpath_husky/model.sdf` | Glue inside Gazebo. JSON `:9002` to SITL. Maps servo channels to `front_left_joint` / right joints (skid steer). |
| **ArduRover SITL** | `scripts/run-ardurover.sh` → `sim_vehicle.py -v Rover --model JSON` | Real ArduRover firmware in software. EKF, arming, GUIDED, cruise speed params in `sim/params/ardurover.parm`. Spawn lat/lon matches the baylands model. |
| **MAVROS** | `mavros/launch/apm.launch` `fcu_url:=udp://:14551@` | ROS ↔ MAVLink. Your API to the firmware. |
| **ros_gz_bridge** | node `parameter_bridge` | One conversion: Gazebo odom → `/ground_truth/odom`. Localization is given. |
| **trajectory_controller_node** | `control.launch.py` | Loads `path_file`, draws markers, runs setup, calls `Control()` at 20 Hz. **Your code lives here** (`ArduroverController`). |
| **path_scorer_node** | `control.launch.py` | 10 Hz samples of odom vs path. Stops at last waypoint within 1 m for 1 s, or 180 s. Writes `paths/score.txt`. |
| **RViz** | `control.launch.py` + `rviz/ugv.rviz` | Fixed frame `map`. Displays `/ground_truth/odom` and `/path_markers`. |
| **QGroundControl** | you start it on the host | Optional. UDP 14550. Useful to confirm arm/mode by hand. |

---

## 6. Setup vs Control

`OnTimer` (20 Hz) does this:

1. Update red line from latest odom.
2. `SetupArdurover()` until it returns true (then it stays true unless mode/arm drop).
3. `Control(odom)` — **this is the assignment.**

Setup (do not redo this in `Control()`):

1. Wait for arming + set_mode services and `/mavros/state.connected`.
2. Set parameter `mav_frame=BODY_NED` on `/mavros/setpoint_velocity`.
3. Call `/mavros/set_mode` until `state.mode == "GUIDED"`.
4. Call `/mavros/cmd/arming` until `state.armed`.

`BODY_NED` (rover body, NED-style axes): `linear.x` forward, `linear.y` right, `angular.z` yaw rate. If it goes backward or steers the wrong way, flip that sign once.

---

## 7. Two odom topics

| Topic | Source | Use |
|---|---|---|
| `/ground_truth/odom` | Gazebo → ros_gz_bridge | **Use this.** Control, score, RViz, path recording. |
| `/mavros/local_position/odom` | SITL EKF via MAVROS | Firmware’s estimate. Noisier, different frame. Ignore for this assignment. |

---

## 8. What not to confuse

- **Gazebo ≠ ArduPilot.** Window open only means physics is running. Motion from your node needs SITL in GUIDED + armed.
- **RViz is not in the loop.** Killing RViz does not stop the rover.
- **`cmd_vel` is TwistStamped**, not Twist. MAVROS also has `cmd_vel_unstamped` (Twist). Wrong type → no connection, rover sits.
- **You talk ROS; firmware talks MAVLink.** MAVROS is the translator.

```bash
ros2 topic echo /mavros/state --once              # armed + mode: GUIDED
ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once
ros2 topic echo /ground_truth/odom --once
```
