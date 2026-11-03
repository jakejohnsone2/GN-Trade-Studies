# G&N mission simulation

Modular start-to-marker simulation for verifying the G&N error budget. Every
block is a function handle in `default_blocks.m`; owners replace their own
handle and leave the rest alone. All numbers live in `params.m`.

## Run it

```matlab
cd Simulation
run_mission        % one mission, plot + error split
run_monte_carlo    % 200 missions x 2 feedback modes + worst-case marker ring
```

## Blocks

| Block | Folder | Owner | Signature |
| --- | --- | --- | --- |
| Marker | `Marker/` | G&N lead | `[m_est, Sigma] = marker_model(m_true, p)` |
| Planner | `Planner/` | Planner | `path = plan_path(map, start_xy, goal_xy, p)` |
| Sensors | `Sensors/` | Sensors | `z = sensor_suite(pose, pose_prev, k, dt, p)` |
| Estimator | `Estimator/` | Estimator | `[x_hat, P] = ekf_step(x_hat, P, z, dt, p)` |
| Plant | `DnC_Stubs/` | D&C | `x = dnc_plant_stub(x, u, dt, p)` |
| Controller | `DnC_Stubs/` | D&C | `[u, cstate] = dnc_controller_stub(x_fb, path, cstate, p)` |
| Loop, errors | `Core/` | G&N lead | `res = simulate_mission(p, blocks)` |

Conventions:

- Pose is `[x; y; theta]` in the shared ENU frame (m, m, rad from East).
- The plant state can be any length; `x(1:3)` must be the pose.
- `u` is opaque to G&N. Only the controller and plant read it.
- The estimator predicts from measured odometry, never from `u`, so it does
  not depend on the D&C model.
- The planner's last waypoint is the exact marker estimate, not a grid node.

## Error split

Logged at every stop (`Core/compute_errors.m`):

```
total = e_marker + e_stop + e_nav
e_marker = goal - true marker          UAS, 15 cm hard bound
e_stop   = what controller saw - goal  D&C, 13 cm (perfect knowledge)
e_nav    = true - what controller saw  G&N, 6 cm
```

`p.sim.feedback = 'truth'` feeds the controller the true pose (perfect
knowledge, D&C alloc). `'estimate'` feeds it `x_hat` (closed loop, 3.2.3.2).
`run_monte_carlo` runs every seed both ways; the difference is the coupling.

## Swapping in a block

```matlab
blocks = default_blocks();
blocks.plant      = @dnc_plant_real;      % D&C delivers these
blocks.controller = @dnc_controller_real;
res = simulate_mission(params(), blocks);
```
