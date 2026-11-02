function blocks = default_blocks()
%DEFAULT_BLOCKS Function handles for every block in the mission loop.
%   Each block owner replaces one handle with their own function. The
%   signatures below are the interface; keep them, change the inside.
%
%   G&N blocks
%     marker     [m_est, Sigma]   = f(m_true, p)
%     planner    path             = f(map, start_xy, goal_xy, p)
%     sensors    z                = f(pose_true, pose_prev, k, dt, p)
%     estimator  [x_hat, P]       = f(x_hat, P, z, dt, p)
%
%   D&C blocks (stand-ins until IF-DC.2 is delivered)
%     plant_init x                = f(start_pose, p)
%     plant      x                = f(x, u, dt, p)
%     controller [u, cstate]      = f(x_fb, path, cstate, p)
%
%   Conventions
%     pose is [x; y; theta] in the shared ENU frame (m, m, rad from East).
%     The plant state x may be any length, but x(1:3) must be the pose.
%     u is opaque to G&N: only the controller and plant read it.
%     The controller sets cstate.done = true once it has commanded the stop.

blocks.marker     = @marker_model;
blocks.planner    = @plan_path;
blocks.sensors    = @sensor_suite;
blocks.estimator  = @ekf_step;

blocks.plant_init = @dnc_plant_init_stub;
blocks.plant      = @dnc_plant_stub;
blocks.controller = @dnc_controller_stub;
end
