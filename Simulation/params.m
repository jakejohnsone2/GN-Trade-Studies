function p = params()
%PARAMS Single source of truth for every number in the G&N simulation.
%   Requirement IDs (GN.x, SYS.x, PT.x, IF-x) refer to the Rover G&N
%   Requirements doc. Change numbers here, never inside a block.

%% ============================ Simulation ================================
p.sim.dt        = 0.01;         % (s) simulation step (100 Hz)
p.sim.t_max     = 180;          % (s) give up after this long
p.sim.settle    = 1.0;          % (s) keep simulating after the stop command
p.sim.feedback  = 'estimate';   % controller input: 'estimate' (x_hat) or 'truth'
p.sim.seed      = 1;            % rng seed for a single run

%% ============================ Rover =====================================
p.rover.half_width = 0.15;      % (m) SYS.1: rover fits in a 0.3 m box
p.rover.v_max      = 0.30;      % (m/s) PT.4 nominal forward speed
p.rover.omega_max  = 1.0;       % (rad/s) pivot rate limit, placeholder

% Stand-in plant only. D&C replaces these with their model (IF-DC.2).
p.rover.tau_v      = 0.30;      % (s) forward speed response time constant
p.rover.tau_w      = 0.15;      % (s) yaw rate response time constant
p.rover.slip_frac  = 0.03;      % (-) wheel slip, fraction of speed (1 sigma)
p.rover.yaw_slip   = 0.02;      % (rad/s) yaw disturbance (1 sigma)

%% ============================ Map (M.1, known beforehand) ===============
p.map.size         = 5;                     % (m) square area of interest
p.map.start        = [0.30, 0.30, 0];       % (m, m, rad) start pose
p.map.obstacles    = [1.5 1.2 0.30;         % (m) [x y radius] per row
                      2.5 3.0 0.40;
                      3.6 1.8 0.35;
                      1.2 3.5 0.25;
                      4.0 3.4 0.20];
p.map.grid_spacing = 0.05;                  % (m) planner grid
p.map.clearance    = p.rover.half_width + 0.10;  % (m) GN.3 clearance C, TBD

%% ============================ Marker (GN.2, IF-UAS.1) ===================
p.marker.true_pos   = [4.5, 4.2];   % (m) where the marker really is
p.marker.bound      = 0.15;         % (m) UAS hard bound on marker error
p.marker.mode       = 'gaussian';   % 'gaussian', 'worst_ring' or 'uniform'
p.marker.sigma      = p.marker.bound / 2.45;  % (m) per axis, 95% at the bound
p.marker.ring_angle = 0;            % (rad) offset direction for 'worst_ring'

%% ============================ Guidance (GN.3, GN.4) =====================
p.guid.L_f        = 0.50;        % (m) minimum straight final approach
p.guid.v_f        = 0.10;        % (m/s) speed at the goal on final approach
p.guid.v_min      = 0.05;        % (m/s) slowest commanded speed while driving
p.guid.pivot_tol  = deg2rad(2);  % (rad) heading error to end a pivot
p.guid.k_heading  = 2.0;         % (1/s) heading gain while driving
p.guid.k_cross    = 4.0;         % (1/m) cross-track gain while driving

%% ============================ Sensors (trade study) =====================
% Placeholder values until the sensor trade picks hardware.
p.sens.rtk_rate    = 10;            % (Hz) RTK position fix rate
p.sens.sigma_rtk   = 0.015;         % (m) RTK fixed, 1 sigma per axis
p.sens.mag_rate    = 10;            % (Hz) magnetometer heading rate
p.sens.sigma_mag   = deg2rad(3);    % (rad) heading, 1 sigma
p.sens.sigma_v     = 0.01;          % (m/s) encoder speed noise, 1 sigma
p.sens.sigma_w     = deg2rad(1);    % (rad/s) gyro rate noise, 1 sigma

%% ============================ Estimator (GN.1) ==========================
p.est.P0 = diag([0.02^2, 0.02^2, deg2rad(5)^2]);   % initial covariance

%% ============================ Error budget (SYS.4) ======================
% 95% horizontal radii. See "Parent requirement and error budget".
p.budget.total  = 0.30;     % (m) SYS.4
p.budget.marker = 0.15;     % (m) IF-UAS.1, hard bound
p.budget.rover  = 0.15;     % (m) GN.0, closed loop
p.budget.nav    = 0.06;     % (m) GN.1
p.budget.stop   = 0.13;     % (m) IF-DC.1, perfect knowledge
end
