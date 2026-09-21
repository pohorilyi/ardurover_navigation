# Path-Following Controller

This note describes the tracking law in `ArduroverController::Control()`. The rover is a differential-drive Husky commanded through ArduRover GUIDED mode. Localization is ground-truth odometry (`/ground_truth/odom`). The controller runs at 20 Hz.

## Commands

After arming and switching to GUIDED, each tick publishes a `geometry_msgs/TwistStamped` on `/mavros/setpoint_velocity/cmd_vel` with MAVROS frame `BODY_NED`:

| Field | Meaning | Units |
|---|---|---|
| `twist.linear.x` | forward speed $v_x$ | m/s |
| `twist.angular.z` | yaw rate $\omega_z$ | rad/s |

All other twist components are zero. ArduRover maps these setpoints to wheel PWM; the companion computer never talks MAVLink directly.

## Principle

The tracker is **pure pursuit plus a PD heading loop**, not a full PID on pose.

1. Walk a monotonic index along the recorded polyline (do not snap to the nearest sample).
2. Place a look-ahead “carrot” a few metres along that unfolding.
3. Point the body at the carrot with a PD yaw-rate command.
4. Command forward speed from a cruise value, bled down into ends, bends, and 180° folds.
5. Treat recorded 180° folds as discrete maneuvers (stop or spin), not as a carrot-behind-us problem.
6. If the chassis hangs on a boardwalk lip, overwrite $v_x,\omega_z$ with a reverse-then-drive recovery.

Stanley-style cross-track feedback is not used for steering. Signed CTE is logged and mixed in only while driving off a lip.

## Progress on the polyline

A waypoint is $(x,y,\psi)$. Progress index $i$ is the start of the current segment. The unclamped projection of the pose onto segment $\mathbf{a}\to\mathbf{b}$ is

$$
t = \frac{(\mathbf{p}-\mathbf{a})\cdot(\mathbf{b}-\mathbf{a})}{\|\mathbf{b}-\mathbf{a}\|^2}.
$$

$i$ increments only when $t > 1$ (the rover has passed the vertex). Overlapping reverse tails are never jumped by “closest point.” A short catch-up search may skip a recorded wiggle, but it never crosses an End/Turn cusp.

Signed cross-track error (positive = left of travel) is

$$
e_{\mathrm{cte}} = \frac{(b_x-a_x)(p_y-a_y) - (b_y-a_y)(p_x-a_x)}{\|\mathbf{b}-\mathbf{a}\|}.
$$

## Look-ahead (carrot)

Default look-ahead is $L_0 = 1.4$ m. The carrot is interpolated along remaining chord length, and it **stops at the next hard cusp** so a 1.4 m look-ahead cannot sit on a recorded rollback.

Heading error is the wrapped bearing to the carrot:

$$
\psi_d = \mathrm{atan2}(y_t - y, x_t - x), \qquad
\psi_e = \mathrm{wrap}(\psi_d - \psi) \in (-\pi,\pi].
$$

In a bend the carrot is shortened so the rover does not cut vertices:

$$
\beta = \mathrm{clip}\left(\frac{\vert\psi_e\vert}{0.7}, 0, 1\right), \qquad
L = L_0 + (0.6 - L_0)\beta.
$$

If the carrot still lies behind the rover ($\vert\psi_e\vert > \pi/2$), $L$ is grown in 0.5 m steps until the target is in the front half-plane. Unstick uses a longer carrot ($L = 2.8$ m) without advancing $i$.

## Yaw command (PD)

Yaw rate is PD on heading error, damped with **measured** yaw rate rather than $\dot{\psi_e}$. Differentiating the carrot bearing is noisy and pumps weave.

$$
\omega_{\mathrm{pd}} = K_p \psi_e - K_d \omega_{\mathrm{meas}}, \qquad
\omega_z = \mathrm{clip}(\omega_{\mathrm{pd}}, -\omega_{\max}, \omega_{\max})
$$

with $K_p = 1.0$, $K_d = 0.08$, $\omega_{\max} = 0.6$ rad/s. There is no integral term: GUIDED already holds a rate command, and an integrator on $\psi_e$ would wind up against saturation and against a hung wheel.

## Forward speed

Cruise is $v_c = 1.0$ m/s. Speed is scaled by remaining polyline length $s_{\mathrm{rem}}$ (4 m bleed) and by distance to the next hard cusp (1.5 m bleed, floor 0.3), then by the bend factor:

$$
s_{\mathrm{end}} = \mathrm{clip}(s_{\mathrm{rem}}/4, 0, 1), \qquad
s_{\mathrm{bend}} = 1 - (1-0.35)\beta, \qquad
v_x = v_c\, s_{\mathrm{end}}\, s_{\mathrm{bend}}.
$$

If $\vert\psi_e\vert > 0.85$ rad (~49°), $v_x = 0$: spin in place rather than drive off the pathwalk. At a heading-180 cusp, $v_x = 0.15$ m/s (creep) while the PD loop aligns yaw.

The stop condition is the **end of the current unfolding** (last vertex, or last segment with $t \ge 1$), not crow-flies distance to `path.back()`. Once latched, the controller publishes zeros and stays stopped.

## Cusps (recorded 180° folds)

A vertex is a cusp when incoming and outgoing tangents point nearly opposite ways:

$$
\hat{\mathbf{t}}_{\mathrm{in}} \cdot \hat{\mathbf{t}}_{\mathrm{out}} < -0.5
$$

Classification uses **recorded yaw**, not vehicle heading:

| Kind | Recording | Action |
|---|---|---|
| **End** | Yaw unchanged, XY after the fold goes the other way (~1 m rollback on paths 0/1) | Stop. Do not track the reverse tail. |
| **Turn** | Recorded yaw flips ~180° | Spin toward the recorded outgoing yaw with creep $v_x$; leave when heading error is below 0.25 rad. |
| **Pass** | Small recorded wiggle | Drive through; carrot and progress ignore it as a wall. |

## Unstick (boardwalk hang)

If the rover has already moved this run, is commanding motion, has small heading error, and XY/body rates stay frozen for 1.5 s, the tracker is interrupted:

1. **Reverse** ($v_x = -0.45$ m/s): alternate $\omega_z = \pm\omega_{\max}$ every 0.4 s so left then right wheels take the load. Continue at least 2 s, then until 0.30 m of reverse or 4 s timeout.
2. **Drive** ($v_x = 0.55$ m/s): PD on heading, mixed 50/50 toward the path if $\vert e_{\mathrm{cte}}\vert > 0.15$ m. Leave when ~0.40 m of forward motion and heading is within ~23°, or after 2 s if already rolling / tilt is falling / attempts exhausted (max 4 reverse→drive cycles).

A 1.5 s cooldown prevents retriggering. Intentional in-place turns ($\vert\psi_e\vert > 0.85$ rad) are not treated as hangs.

## Tick order

Each 20 Hz step is: advance $i$ → update End/Turn mode → place carrot → compute $v_x,\omega_z$ → possibly overwrite with unstick → always publish (GUIDED needs a stream). Implementation: `path_follow.hpp` (geometry and tracking), `unstick.hpp` (recovery constants), `ardurover_controller.cpp` (`Control()`).
