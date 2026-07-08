# mppi_planner_node

Standalone MPPI-style path tracking node for the AAC2026 Ackermann vehicle.

This package intentionally avoids Nav2 dependencies. It does not use
`nav2_core`, `nav2_util`, lifecycle nodes, pluginlib, controller server, or a
Nav2 costmap. Instead, it plugs into the existing AAC2026 interface:

- subscribes: `/local_path/body` (`nav_msgs/msg/Path`)
- publishes: `/ackermann_auto` (`ackermann_msgs/msg/AckermannDriveStamped`)
- publishes debug trajectory: `/mppi/optimal_trajectory`
- publishes RViz markers: `/mppi/trajectory_markers`
- publishes tuning status: `/mppi/debug_status`

The planner samples speed and steering command sequences around a warm-started
nominal control sequence, rolls them out with a kinematic bicycle model, scores
them against the body-frame local path, and applies the MPPI weighted update.
The marker topic contains sampled rollout line strips plus the selected
trajectory, which makes late corner-entry behavior visible in RViz.
Sampled rollouts are cost-colored, the best sampled rollout is drawn as a
thicker yellow line, and the final MPPI command trajectory is drawn as a green
line.
It also includes corner diagnostics: white projection, yellow heading
lookahead, orange curvature lookahead, and a text marker showing corner heading
change, target speed, and whether the command speed was limited.

## Run

```bash
ros2 launch mppi_planner_node mppi_planner.launch.py
```

Use it instead of `pure_pursuit_node` or `stanley_node`, because all three
publish `/ackermann_auto`.

After rebuilding or changing the launch pane, run the pre-drive checks:

```bash
ros2 run mppi_planner_node mppi_pre_drive_check.py --expect-live-profile early --min-live-command-gain 0.10
```

This checks that the running MPPI process is fresh, then probes corner behavior
on isolated private topics without touching `/local_path/body` or
`/ackermann_auto`. The individual checks are also available:

```bash
ros2 run mppi_planner_node mppi_runtime_check.py --expect-profile early --min-steering-feedforward-command-gain 0.10
ros2 run mppi_planner_node mppi_corner_probe.py
ros2 run mppi_planner_node mppi_visualization_check.py
```

`mppi_runtime_check.py` fails fast when it detects a deleted or mismatched
planner executable, so stale launch panes are obvious before driving. Add
`--check-parameters-on-executable-error` only when you also want to inspect
whatever parameters the stale process still exposes.

To verify a preset's synthetic corner behavior before applying it to the
vehicle node:

```bash
ros2 run mppi_planner_node mppi_pre_drive_check.py --profile aggressive
```

The probe publishes a synthetic curved local path, then checks command output,
optimal trajectory, rollout markers, diagnostic markers, and corner speed debug
fields.
The visualization check is for a running node that is already receiving local
paths; it verifies `mppi_rollouts`, `mppi_best_sampled`, `mppi_optimal`, and
`mppi_diagnostics` on `/mppi/trajectory_markers`, including non-empty
LINE_STRIP markers and the corner/steering diagnostic text marker.
To compare a preset without touching the real `/ackermann_auto` topic, apply it
only to the isolated probe node:

```bash
ros2 run mppi_planner_node mppi_corner_probe.py --profile aggressive
ros2 run mppi_planner_node mppi_corner_probe.py --profile baseline
```

To compare several presets and synthetic corner curvatures before driving:

```bash
ros2 run mppi_planner_node mppi_profile_sweep.py
ros2 run mppi_planner_node mppi_profile_sweep.py --profiles early,aggressive,slow_corner --curvatures 0.12,0.18,0.28
```

The sweep prints a recommended `mppi_drive_trial.py --profile ...` command
based on speed target reduction and steering response in the synthetic corners.
The default scoring is biased toward fixing late corner entry; use
`--objective balanced` if the vehicle already turns early enough and you mainly
want to avoid twitchiness. Use the recommendation as a starting point, then
trust the real drive-trial log over the synthetic score.

## Notes

This is a path tracker, not a full Nav2 controller server replacement. Obstacle
avoidance/costmap terms are not included; provide an obstacle-aware local path
upstream if those costs are needed.

The default config is biased toward earlier corner entry: stronger heading
lookahead, stronger terminal heading cost, more steering exploration, and less
progress reward. It also adds a curvature-based steering feedforward cost so
rollouts that begin steering into the visible corner are preferred earlier. It
blends the first command slightly toward that curvature steering target, which
helps short local paths start turning before the stochastic optimizer has fully
warmed up. It
also limits the first speed command toward the current
corner speed target when a turn is already visible in the local path. The
rollout speed cost tracks that corner speed target instead of the straight
target speed while a corner is visible, so MPPI itself prefers slowing earlier.
The corner target also respects a lateral acceleration limit, which gives a
physical knob for how fast the vehicle should enter tight turns. If corner
entry is still late, move to `aggressive`. If the vehicle becomes twitchy, move
back with `stable` or `baseline`.

Most controller, cost, sampling, and visualization parameters can be changed
while the node is running:

```bash
ros2 param set /mppi_planner_node terminal_heading_weight 6.0
ros2 param set /mppi_planner_node heading_weight 3.5
ros2 param set /mppi_planner_node heading_lookahead_weight 3.0
ros2 param set /mppi_planner_node heading_lookahead_distance 1.5
ros2 param set /mppi_planner_node steering_feedforward_weight 0.25
ros2 param set /mppi_planner_node steering_feedforward_lookahead_distance 1.4
ros2 param set /mppi_planner_node steering_feedforward_command_gain 0.20
ros2 param set /mppi_planner_node tracking_min_command_speed 0.25
ros2 param set /mppi_planner_node curvature_speed_weight 1.2
ros2 param set /mppi_planner_node curvature_speed_gain 1.0
ros2 param set /mppi_planner_node lateral_accel_limit 0.45
ros2 param set /mppi_planner_node use_lateral_accel_speed_limit true
ros2 param set /mppi_planner_node use_corner_speed_as_speed_target true
ros2 param set /mppi_planner_node limit_command_speed_to_corner true
ros2 param set /mppi_planner_node corner_speed_margin 0.05
ros2 param set /mppi_planner_node steering_noise_std 0.40
ros2 param set /mppi_planner_node progress_weight 0.5
```

The same tuning sets are available as presets:

```bash
ros2 run mppi_planner_node mppi_tune.py early
ros2 run mppi_planner_node mppi_tune.py aggressive
ros2 run mppi_planner_node mppi_tune.py slow_corner
ros2 run mppi_planner_node mppi_tune.py stable
ros2 run mppi_planner_node mppi_tune.py baseline
```

The launch defaults match `early`. Use `aggressive` if corner entry is still
late. Use `slow_corner` if the vehicle still enters tight turns too fast after
`aggressive`. Use `stable` if the vehicle starts oscillating or oversteering.
Use `baseline` to return to the original conservative settings.
Presets are applied through the atomic parameter service so the planner does
not run with a partially-applied tuning set. Each update waits up to 8 seconds
by default; use `--timeout 15` if the ROS graph is slow to respond. Use
`--sequential` to fall back to individual `ros2 param set` calls.
The active preset name is stored in `tuning_profile` and is included in
`/mppi/debug_status` and CSV logs.
The tuning command accepts either a positional profile or `--profile`, so these
are equivalent:

```bash
ros2 run mppi_planner_node mppi_tune.py aggressive
ros2 run mppi_planner_node mppi_tune.py --profile aggressive
```

For manual tuning during a session, add `--verify` so the runtime check confirms
the active `tuning_profile` and profile-specific command feedforward gain before
you drive:

```bash
ros2 run mppi_planner_node mppi_tune.py aggressive --verify
```

If the vehicle starts oscillating:

```bash
ros2 param set /mppi_planner_node steering_noise_std 0.28
ros2 param set /mppi_planner_node heading_lookahead_weight 1.5
ros2 param set /mppi_planner_node steering_feedforward_command_gain 0.10
ros2 param set /mppi_planner_node curvature_speed_weight 0.4
ros2 param set /mppi_planner_node steering_rate_weight 0.12
ros2 param set /mppi_planner_node max_steering_rate 3.0
```

Topic parameters and `update_rate` still require a node restart because they
change ROS interfaces or timer construction.

For live tuning, keep this open in another terminal:

```bash
ros2 run mppi_planner_node mppi_watch.py
```

To print a concrete next tuning command with each hint:

```bash
ros2 run mppi_planner_node mppi_watch.py --commands
```

To save a CSV log for post-drive tuning:

```bash
ros2 run mppi_planner_node mppi_log_debug.py -o /tmp/mppi_debug_drive.csv
```

After the run, analyze the CSV:

```bash
ros2 run mppi_planner_node mppi_analyze_log.py /tmp/mppi_debug_drive.csv --commands
```

To turn the CSV into a concrete next tuning action:

```bash
ros2 run mppi_planner_node mppi_suggest_tune.py /tmp/mppi_debug_drive.csv
ros2 run mppi_planner_node mppi_suggest_tune.py /tmp/mppi_debug_drive.csv --assume-late
```

To compare several drive trials:

```bash
ros2 run mppi_planner_node mppi_compare_trials.py /tmp/mppi_drive_trial_*.csv
```

For a bounded drive trial that applies a profile, records `/mppi/debug_status`,
analyzes the result, and prints a concrete next tuning action:

```bash
ros2 run mppi_planner_node mppi_drive_trial.py --profile aggressive --duration 60 --assume-late --check-visualization
ros2 run mppi_planner_node mppi_drive_trial.py --profile slow_corner --duration 60 --assume-late --check-visualization
```

When `--profile` is used, the trial re-runs `mppi_runtime_check.py` after the
profile update and verifies that the requested `tuning_profile` and its
`steering_feedforward_command_gain` are active before logging.

For raw JSON debug output:

```bash
ros2 topic echo /mppi/debug_status
```

The watcher summarizes selected command speed, estimated corner heading change,
corner speed target, tracking speed floor, steering target/error, whether the
speed command was corner-limited or floor-adjusted, rollout count, whether the
lateral acceleration target is
active, early steering lag ratio, selected-vs-best-sampled cost gap, and a
tuning hint for the current rolling window. With `--commands`, it can suggest `iteration_count`,
`temperature`, `max_accel`, `corner_speed_margin`, `lateral_accel_limit`,
`max_steering_rate`, or preset changes. The raw JSON reports the full selected
cost, sampled rollout cost range, path heading/lookahead heading, curvature
lookahead distance, steering target/error, best sampled rollout cost, and the
main cornering weights currently being used. The CSV logger records the same
corner target, command, cost, steering, and rollout summary fields for later
comparison. The analyzer summarizes corner samples, speed-over-target error,
steering response, early steering lag, optimizer gap, limiter activity, rollout
counts, final tuning values such as feedforward gain and steering-rate limit,
and a recommended next tuning command.
It also reports the commanded-speed lateral acceleration estimate, which is a
useful sanity check against `lateral_accel_limit` when corner entry is still
late or too fast.
If the corner speed limit is active but the command speed remains high, increase
`max_accel` to allow faster deceleration or lower `corner_speed_margin`.
If heading error is large and the command speed collapses toward zero, raise
`tracking_min_command_speed` to about `0.25` and restart the MPPI launch pane if
the parameter is not declared.
If the vehicle is still too fast entering tight turns, lower
`lateral_accel_limit`; if it becomes too slow, raise it slightly.
