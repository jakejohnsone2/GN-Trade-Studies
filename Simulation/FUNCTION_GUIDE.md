# Function Guide

This document explains the MATLAB functions in this folder, their arguments and outputs, how they connect during a mission, and what to change when replacing a stand-in or adapting the simulation. The code is the source of truth if this guide and implementation ever differ.

## Mission flow

`run_mission.m` builds parameters and block handles, then calls `simulate_mission`. The simulator seeds the random number generator, asks the marker block for a reported goal, asks the planner for a path, initializes the plant and estimated pose, and runs the closed loop. Each time step calls the controller, plant, sensors, and estimator in that order. At the end, it computes the error split and returns logs and results. `run_monte_carlo.m` repeats this process across seeds and feedback modes.

```text
params + default_blocks
        |
        v
marker_model -> plan_path -> controller -> plant -> sensor_suite -> ekf_step
                    ^              |          |                         |
                    +--------------+----------+-------------------------+
                                   repeated each time step
        |
        v
compute_errors -> results -> plot_mission / Monte Carlo report
```

Pose is `[x; y; theta]` in ENU coordinates, with meters and radians measured from East. Planner waypoints are rows `[x y]`. The plant state may have any length, but its first three elements must always be the pose. The command `u` is private to the controller and plant.

## Entry points and configuration

### `setup_paths()` — `setup_paths.m`

Adds this folder and all subfolders to the MATLAB search path using the location of the file itself. It has no inputs or outputs. Usually no changes are needed. If files are moved into a different folder layout, preserve recursive path setup or update it to include the new block folders.

### `params()` — `params.m`

Returns the single parameter structure `p`, organized under `sim`, `rover`, `map`, `marker`, `guid`, `sens`, `est`, and `budget`. It takes no arguments. This is the place to change simulation time step/limits/feedback/seed, rover and map geometry, marker error model, guidance behavior, sensor rates/noise, initial estimator covariance, and error budgets. Keep units and field names aligned with all functions that read them. The stand-in plant fields `tau_v`, `tau_w`, `slip_frac`, and `yaw_slip` can be removed or replaced when the real plant no longer uses them. Sensor and marker values are placeholders where noted in comments.

### `default_blocks()` — `default_blocks.m`

Returns `blocks`, a structure of function handles. Its fields and required signatures are:

| Field | Signature | Role |
| --- | --- | --- |
| `marker` | `[m_est, Sigma] = f(m_true, p)` | Report marker estimate and covariance |
| `planner` | `path = f(map, start_xy, goal_xy, p)` | Plan waypoints to the estimated marker |
| `sensors` | `z = f(pose, pose_prev, k, dt, p)` | Generate measurements from motion |
| `estimator` | `[x_hat, P] = f(x_hat, P, z, dt, p)` | Predict and correct pose estimate |
| `plant_init` | `x = f(start_pose, p)` | Initialize plant state |
| `plant` | `x = f(x, u, dt, p)` | Advance plant one time step |
| `controller` | `[u, cstate] = f(x_fb, path, cstate, p)` | Command motion and report stop status |

To swap in an implementation, assign its handle here or in a caller after `default_blocks()`. Keep the signature and data conventions stable. The real plant, controller, and plant initializer replace the D&C stubs when delivered. The planner, marker, sensor, and estimator handles can be replaced independently.

### `run_mission.m`

Script, not a function. Clears the MATLAB session state/figures, sets paths, builds `p` and `blocks`, calls `simulate_mission`, prints one mission's error terms and success, and plots it. Change the script when you want a different single-run setup, custom blocks, or different report output. It currently uses `params()` defaults.

### `run_monte_carlo.m`

Script. Runs `N` Gaussian-marker trials for both `'truth'` and `'estimate'` feedback, then `n_ring` closed-loop trials with the marker error at the hard bound. It reports 95th percentile error radii, budget passes, coupling, success, and timeouts, then plots rover error histograms. Change `N` and `n_ring` for run size/resolution; adjust report checks if requirements or budget definitions change. Each trial's seed is set before calling the simulator. Local helpers below are part of this script.

## Core functions

### `simulate_mission(p, blocks)` — `Core/simulate_mission.m`

Runs one mission and returns `res`, containing time, true and estimated pose histories, covariance diagonals, planned path, true/reported marker, feedback mode, timeout flag, error components, and success.

Inputs: `p` from `params()`; `blocks` from `default_blocks()` or a compatible custom structure. It seeds RNG from `p.sim.seed`. It calls `blocks.marker(m_true,p)`, then `blocks.planner(p.map,start_xy,m_est,p)`, initializes truth via `blocks.plant_init`, and initializes `x_hat` by adding a random draw based on `p.est.P0` to the start pose. Each loop iteration passes either `x_hat` or true pose to the controller based on `p.sim.feedback`, advances plant, generates measurements, and updates the estimate. It keeps simulating for `p.sim.settle` after `cstate.done` to capture residual motion.

When integrating custom blocks, preserve output dimensions and the controller's `cstate.done` completion flag. The controller's `x_fb` is three elements; the plant's first three state elements are pose. `p.sim.dt`, `t_max`, `settle`, `seed`, and `feedback` control this loop. The `goal` for error accounting is the last waypoint, so the planner must put its exact goal there.

### `compute_errors(m_true, goal, fb_final, true_final, hat_final)` — `Core/compute_errors.m`

Returns a struct of 2D error vectors (meters). Inputs are true marker, planned goal, final pose as seen by controller, final true pose, and final estimate. It computes `e_marker = goal - m_true`, `e_stop = fb_final - goal`, `e_nav = true_final - fb_final`, `rover = true_final - goal`, `total = true_final - m_true`, and `est = true_final - hat_final`. These definitions produce the exact vector identity `total = e_marker + e_stop + e_nav`. In truth feedback, `e_nav` is zero by construction; `est` still reports estimator error. Change this function only if the team's error-budget definitions change, and update reports/plots to match.

### `plot_mission(res, p)` — `Core/plot_mission.m`

Draws the obstacle map and clearance rings, planned waypoints, true and estimated tracks, and marker/budget circles. Inputs are the simulator result and parameter structure. It reads `p.map`, `p.budget`, `p.marker` and corresponding fields in `res`. Update it if result/map schemas or displayed budgets change; it does not affect mission dynamics.

### `wrap_pi(a)` — `Core/wrap_pi.m`

Wraps angle value(s) in radians to `(-pi, pi]` using modulo arithmetic. Used by estimator, sensor, controller, and plant to keep heading differences bounded. No changes are normally needed; if changing angle conventions, update every consumer consistently.

## Replaceable blocks and their helpers

### `marker_model(m_true, p)` — `Marker/marker_model.m`

Inputs: true marker position `m_true` (two coordinates; normalized to a column) and `p`. Outputs: `m_est` (2x1 reported position) and `Sigma` (2x2 covariance). `p.marker.mode` selects `worst_ring` (fixed bound and angle), `gaussian` (independent normal axes, resampled until within the hard radius), or `uniform` (uniformly sampled disk). It uses `p.marker.bound`, `ring_angle`, and `sigma`. Replace this function to match the marker localization system; preserve dimensions and update `Sigma` to represent the reported uncertainty. If adding a mode, add the parameter(s) in `params.m` and a matching case.

### `plan_path(map, start_xy, goal_xy, p)` — `Planner/plan_path.m`

Inputs: map structure, start `[x y]`, exact goal `[x y]`, and parameters. Output: `path` with `wp` as an Nx2 waypoint array and `final_ok` indicating whether its last segment meets `p.guid.L_f`. The stand-in samples a grid at `map.grid_spacing`, marks nodes blocked by obstacles plus clearance, makes an 8-connected weighted graph, finds a shortest path between nearest free grid nodes, adds exact start and goal, then removes intermediate points where a direct collision-free segment is possible. Replace the stand-in with the trade-study planner while preserving `path.wp`, start/end semantics, and collision/clearance expectations. The final waypoint must equal `goal_xy`, because error accounting treats it as the target. `is_free`, `nearest_free`, and `segment_free` are local helpers described below.

#### Local planner helpers

- `is_free(xy, map)` returns a logical value per candidate `[x y]` point: true if it is outside every obstacle radius expanded by `map.clearance`.
- `nearest_free(xy, free, q)` returns the index of the free point nearest query `q` in Euclidean distance.
- `segment_free(a, b, map)` samples the line from `a` to `b` at half-grid-spacing intervals and returns true if every sample is free. For a new planner, ensure collision checking is at least as strict as required; sampled checking can miss very thin obstacles if resolution is too coarse.

### `sensor_suite(pose, pose_prev, k, dt, p)` — `Sensors/sensor_suite.m`

Inputs: current and previous true poses, integer step `k`, time step, and `p`. Output `z` has `odom` `[v; omega]` every step, `rtk` `[x;y]` on RTK update steps or empty, and `mag` heading on magnetometer update steps or empty. The stand-in derives forward speed from pose displacement projected along previous heading, derives yaw rate from wrapped heading change, and adds configured Gaussian noise. RTK and magnetometer rates/noise come from `p.sens`. Replace with hardware models or recorded data while preserving these field names/units, update timing, and empty values when a measurement is unavailable. It uses `wrap_pi`.

### `ekf_step(x_hat, P, z, dt, p)` — `Estimator/ekf_step.m`

Inputs: previous estimated 3x1 pose, 3x3 covariance, sensor struct, `dt`, and `p`. Outputs: updated pose and covariance. First it predicts from odometry (`z.odom`) using a planar motion model and process noise derived from speed/yaw sensor variances. Then it corrects from RTK position and magnetometer heading if those fields are nonempty. `correct` is a local Kalman update helper. Replace this estimator if needed while preserving dimensions and sensor contract; model latency/update rates if required by 3.2.2.1. Noise settings are in `params.m`; the initial covariance is `p.est.P0`.

#### Local estimator helper: `correct(x, P, innov, H, R)`

Applies one Kalman correction using innovation, observation matrix, and measurement covariance. Returns corrected state and covariance. It wraps heading and uses Joseph covariance form for numerical symmetry/stability. It is local to `ekf_step`; retain angle handling and covariance dimensions if modifying the filter.

### `dnc_plant_init_stub(start_pose, p)` — `DnC_Stubs/dnc_plant_init_stub.m`

Creates the stand-in plant state `[x;y;theta;v;omega]` with zero initial speeds. Inputs are the initial pose and parameters; output is plant state. Replace alongside the real D&C plant if its state layout differs. The required interface contract is only that the first three state entries are pose and the initializer accepts `(start_pose,p)`.

### `dnc_plant_stub(x, u, dt, p)` — `DnC_Stubs/dnc_plant_stub.m`

Advances the stand-in unicycle plant one step. Inputs: state, opaque command vector, time step, and parameters. The current command is `[v_cmd; omega_cmd]`; first-order speed response uses `tau_v`/`tau_w`, then slip and yaw disturbance are added before integrating pose. Output state retains the same layout. Replace with D&C dynamics and update `plant_init` consistently. Preserve the `(x,u,dt,p)` interface and `x(1:3)` pose requirement; the estimator must continue to use sensor measurements, not commands.

### `dnc_controller_stub(x_fb, path, cstate, p)` — `DnC_Stubs/dnc_controller_stub.m`

Inputs: feedback pose `[x;y;theta]`, planner output, controller memory (`[]` initially), and parameters. Outputs: command `u` and updated controller state. The stub pivots to each straight leg's heading, drives the leg with heading/cross-track correction, ramps speed down on the final leg, and sets `cstate.done` when it believes it has reached the goal. `u` is `[v_cmd;omega_cmd]` for the matching stub plant. Replace with the real controller, preserving the `(x_fb,path,cstate,p)` signature and `cstate.done`; ensure its command output is understood by the selected plant. `x_fb` may be estimate or truth. Guidance gains, speed targets, pivot tolerance, and rover limits are in `params.m`.

#### Local controller helper: `sat(y, lim)`

Clamps scalar command `y` to `[-lim, lim]`. Used for yaw rate limiting. If command saturation moves into a shared utility or actuator model, update the controller accordingly.

## Monte Carlo script helpers

These functions are local to `run_monte_carlo.m` and operate on the script's statistics structure:

- `init_stats(n)` creates arrays for `n` trials: NaNs for numeric metrics and false flags for success/timeouts.
- `record(s, i, res)` extracts norms of each error vector and success/timeout from one mission result into trial `i`.
- `pct95(x)` sorts non-NaN values and returns the empirical element at index `ceil(0.95*N)`. It is not MATLAB's interpolated percentile convention.
- `row(name, val, budget)` prints a formatted result and marks it passing if it does not exceed the supplied budget.

Update these helpers together if `res.err` changes or the Monte Carlo metric definition changes. They do not alter individual mission simulation.

## What to change for common updates

| Goal | Change |
| --- | --- |
| Change scenario, limits, noise, budget, or feedback | Edit `params.m`; keep field names and units consistent |
| Use real D&C hardware/model behavior | Replace `plant_init`, `plant`, and `controller` handles in `default_blocks.m`; preserve interfaces and the first-three-state pose rule |
| Adopt the trade-study planner | Replace `blocks.planner`; return Nx2 `path.wp` with exact goal in the final row |
| Change sensor suite or error assumptions | Replace `sensor_suite`, tune `p.sens`, and ensure `ekf_step` understands the resulting `z` fields |
| Change estimator | Replace `blocks.estimator`; maintain a 3-element pose and 3x3 covariance expected by the logger |
| Change marker error model | Replace `marker_model` or mode logic; keep 2x1 estimate and 2x2 covariance |
| Change requirements/error decomposition | Update `compute_errors`, the budgets in `params.m`, and matching report/plot checks |
| Change only visualization or printed summaries | Edit `plot_mission.m`, `run_mission.m`, or `run_monte_carlo.m` without changing block dynamics |

## Integration checklist

1. Keep function names, argument order, units, and output shapes compatible with `default_blocks.m`.
2. Keep pose as `[x;y;theta]` in ENU; maintain `plant_state(1:3)` as pose.
3. Make the planner's last waypoint exactly the marker estimate passed to it.
4. Have the controller set `cstate.done = true` once it commands a stop; `simulate_mission` uses that flag to stop and to mark timeouts.
5. Keep controller commands and plant interpretation coordinated. G&N treats `u` as opaque.
6. If a function starts using a new numeric assumption, add it to `params.m` rather than burying it in the function.
