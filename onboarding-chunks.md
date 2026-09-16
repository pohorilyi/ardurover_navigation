# Onboarding chunks: ROS / ArduPilot / MAVLink via this assignment

Treat the assignment as the spine. Learn ROS, ArduPilot, and MAVLink only as far as each chunk needs, then stop.

A good shape is **setup → watch the stack → talk to the autopilot → dumb controller → real tracker → polish**.

## Rule of thumb

Each chunk has a **done when** that is visible (a window, a topic, a moving rover, a score file). If you cannot demo it, you are still in that chunk.

Do **not** start with ArduPilot source, Gazebo SDF, or a full ROS course. Those are rabbit holes. You need: one odom topic, one velocity command, GUIDED mode, then a tracking law.

If time is short, **cut learning, not chunks 4–7**. A constant-velocity rover that then becomes Stanley/pure-pursuit on three paths beats a half-read ArduPilot wiki and an empty `Control()`.

---

## Chunk 0 — Machine (half day, blocker)

**Goal:** Linux environment that can run their Docker.

- WSL2 Ubuntu 24.04 (or dual-boot if you already know you want it).
- Docker **inside** Linux, NVIDIA Windows driver if you have an NVIDIA GPU.
- Repo cloned under `~/ardurover_navigation` (not `C:\`).

**Done when:** `./docker/build.sh` finishes and `./docker/run.sh` drops you in a container shell.

**Learn:** almost nothing ROS-related. Just “this task runs in their image.”

---

## Chunk 1 — Sim comes up (half day)

**Goal:** See the rover exist. No controller yet.

Inside the container:

```bash
./scripts/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ardurover_nav sim.launch.py
```

**Done when:** Gazebo opens, rover is in the baylands, no crash loop.

If Gazebo is white/black, fix graphics here. Do not write C++ on a dead sim.

**Learn (thin):**

- Launch file = process supervisor.
- Gazebo = physics. ArduRover SITL = firmware. They are two programs talking over the network.

**Skip:** world SDF, plugin internals, ArduPilot `waf`.

---

## Chunk 2 — ROS as a bus (1–2 hours)

**Goal:** Confirm you can *see* the robot in ROS. Still no controller.

Second terminal (`./docker/attach.sh`):

```bash
ros2 topic list
ros2 topic echo /ground_truth/odom --once
ros2 node list
ros2 service list | grep mavros
```

**Done when:** you can explain, in one sentence each, what `/ground_truth/odom` is and that MAVROS services exist.

**Learn:**

- **Node** = process.
- **Topic** = stream (`odom` is pose + twist).
- **Service** = RPC (`arming`, `set_mode`).
- `ros2 topic echo` is your debugger.

**Skip:** TF trees, QoS deep dive, writing new packages.

This is the whole ROS onboarding you need for the task.

---

## Chunk 3 — Autopilot from the outside (2–3 hours)

**Goal:** Understand what `SetupArdurover()` already did, and how *you* will command motion.

Read, in this order:

1. `src/ardurover_nav/src/ardurover_controller.cpp` — `SetupArdurover()` (WaitServices → BODY_NED → GUIDED → Arm).
2. `src/ardurover_nav/src/trajectory_controller_node.cpp` — timer, odom callback, calls `Control()`.
3. MAVROS velocity setpoint: topic `/mavros/setpoint_velocity/cmd_vel` (`geometry_msgs/Twist`), frame already set to `BODY_NED`.

Optional: QGroundControl on Windows to UDP `14550`, arm in Manual, twitch the joystick. That is ArduPilot as a human.

**Done when:** you know:

- GUIDED = “accept setpoints from companion computer.”
- BODY_NED ≈ forward/right in the rover body (you will confirm the sign of `linear.x` / `angular.z` in the next chunk).
- You will **publish Twist**, not talk MAVLink bytes.

**Learn:**

- ArduPilot **modes** and **arming** (conceptual).
- MAVLink as the radio protocol; MAVROS as the ROS adapter.
- One message: velocity setpoint.

**Skip:** full MAVLink XML, parameter lists, custom modes beyond GUIDED.

---

## Chunk 4 — Smallest moving controller (half day) — first real delivery step

**Goal:** Rover drives *somewhere* from `Control()`. Ignore the path.

In `Control()`:

- Publish a constant slow forward Twist (e.g. `linear.x = 0.5`, rest zero).
- Rebuild, launch `control.launch.py` on `0-drive-straight.path`.

**Done when:** rover arms, goes GUIDED, rolls forward in Gazebo. RViz red line grows.

If it goes backward or spins, you just learned the frame. Fix signs. This chunk is cheap and saves a day of “my PID is wrong” when the command never reached the firmware.

**Learn:** the actual command path: your node → MAVROS → SITL → Gazebo.

---

## Chunk 5 — Track the straight path (main algorithm, ~1 day)

**Goal:** Follow `0-drive-straight.path` and get a decent `paths/score.txt`.

Implement a simple law, not a thesis:

1. Find the closest point / a look-ahead point on the polyline.
2. Heading error + cross-track error.
3. `linear.x` = cruise speed (slow near the end).
4. `angular.z` = P (or P + a bit of CTE), saturated.

Stanley or pure pursuit are enough. PID on heading is enough for path 0.

**Done when:** rover finishes the straight path, score is clearly not ~0 (aim high, but “completes with bounded CTE” is the gate).

**Learn:** path following in *this* coordinate system. That is the robotics core of the assignment.

---

## Chunk 6 — Turns, then the hard path (~1 day)

**Goal:** Same controller, all three launch commands from the README.

Tune, don’t rewrite:

- Look-ahead / speed vs curvature.
- Slow down on sharp turns (`2-complicated.path`).
- Stop condition: last waypoint within ~1 m (scorer already uses that).

**Done when:** all three paths complete; scores are something you would show. Rebuild after each C++ change.

**Learn:** why geometric trackers need speed scheduling. Still not LQR/MPC unless you are bored and already scoring well.

---

## Chunk 7 — Deliverables (half day)

**Goal:** What they asked to submit.

1. **Code** — fork/repo that runs the three `ros2 launch ...` lines.
2. **Report** — 1–2 pages: law (with formulas), Twist you send, BODY_NED, how you pick the target point, gains. That write-up *is* your onboarding notes cleaned up.
3. **Video** — Gazebo + RViz, any path, green vs red.

**Done when:** a stranger can clone, launch path 0, and see tracking.

---

## How this maps to onboarding vs homework

| Chunk | Delivery | Onboarding |
|---|---|---|
| 0–1 | Can run the test | Docker/Gazebo existence |
| 2 | Can debug | ROS topics |
| 3 | Know the API | ArduPilot modes, MAVROS, MAVLink-as-idea |
| 4 | Commands reach the rover | End-to-end stack |
| 5–6 | The actual task | Path tracking |
| 7 | Submit | You can explain it |

## Daily loop once the sim works

```text
edit Control() → ./scripts/build.sh → source install/setup.bash
→ launch control.launch.py with one path
→ watch RViz CTE → read paths/score.txt → tweak
```

Use `ros2 topic echo /mavros/setpoint_velocity/cmd_vel` if the rover does not move.

## What not to turn into a course

- Writing Gazebo plugins or a new ROS package
- Custom MAVLink messages
- Localization / EKFs (they gave you ground truth)
- QGC path recording except as a curiosity (`sim.launch.py` + `path_recorder_node`)
- Perfecting Docker/WSL after the 3D view works
