function res = simulate_mission(p, blocks)
%SIMULATE_MISSION Run one start-to-marker mission through every block.
%   res = simulate_mission(params(), default_blocks())
%
%   Loop each step:  controller -> plant -> sensors -> estimator.
%   p.sim.feedback chooses what the controller sees: 'estimate' (x_hat, the
%   real closed loop, 3.2.3.2) or 'truth' (perfect knowledge, D&C alloc). Same
%   seed, both modes: the difference is the estimator/controller coupling.

rng(p.sim.seed);
dt = p.sim.dt;

%% Mission setup
m_true        = p.marker.true_pos(:);
[m_est, Sig]  = blocks.marker(m_true, p);
path          = blocks.planner(p.map, p.map.start(1:2), m_est, p);
goal          = path.wp(end,:)';

x_true = blocks.plant_init(p.map.start, p);
x_hat  = p.map.start(:) + sqrtm(p.est.P0) * randn(3,1);
P      = p.est.P0;
cstate = [];

%% Logs
n_max  = ceil(p.sim.t_max / dt);
log_t  = nan(1, n_max);
log_x  = nan(3, n_max);
log_xh = nan(3, n_max);
log_P  = nan(3, n_max);

%% Main loop
use_truth   = strcmp(p.sim.feedback, 'truth');
settle_left = round(p.sim.settle / dt);
k = 0;
while k < n_max
    k = k + 1;
    pose_prev = x_true(1:3);

    if use_truth, x_fb = x_true(1:3); else, x_fb = x_hat; end
    [u, cstate] = blocks.controller(x_fb, path, cstate, p);
    x_true      = blocks.plant(x_true, u, dt, p);
    z           = blocks.sensors(x_true(1:3), pose_prev, k, dt, p);
    [x_hat, P]  = blocks.estimator(x_hat, P, z, dt, p);

    log_t(k)    = k * dt;
    log_x(:,k)  = x_true(1:3);
    log_xh(:,k) = x_hat;
    log_P(:,k)  = diag(P);

    if cstate.done
        settle_left = settle_left - 1;
        if settle_left <= 0, break; end
    end
end

if use_truth, x_fb = x_true(1:3); else, x_fb = x_hat; end

%% Results
res.t         = log_t(1:k);
res.x_true    = log_x(:,1:k);
res.x_hat     = log_xh(:,1:k);
res.P_diag    = log_P(:,1:k);
res.path      = path;
res.m_true    = m_true;
res.m_est     = m_est;
res.m_Sigma   = Sig;
res.feedback  = p.sim.feedback;
res.timed_out = ~cstate.done;
res.err       = compute_errors(m_true, goal, x_fb(1:2), x_true(1:2), x_hat(1:2));
res.success   = ~res.timed_out && norm(res.err.total) <= p.budget.total;
end
