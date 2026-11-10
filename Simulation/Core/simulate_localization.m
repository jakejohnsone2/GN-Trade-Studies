function res = simulate_localization(p, blocks, traj)
%SIMULATE_LOCALIZATION Run sensors + estimator along a scripted trajectory.
%   res = simulate_localization(params(), default_blocks(), ...
%                               scripted_trajectory('square', params()))
%
%   Open loop: no controller, no plant, no D&C model. The true pose comes
%   from traj, so the estimator error is purely a sensor/estimator result
%   (3.2.2.4 position, 3.2.2.2 heading, 3.2.2.6 lock time).
%
%   Uses the same blocks.sensors and blocks.estimator handles as
%   simulate_mission, so swapping a sensor model or filter here swaps it
%   in the full mission too.

rng(p.sim.seed);
dt = traj.t(2) - traj.t(1);
N  = numel(traj.t);

x_hat = traj.pose(:,1) + sqrtm(p.est.P0) * randn(3,1);
P     = p.est.P0;
n     = numel(x_hat);

log_xh = nan(n, N);
log_P  = nan(n, N);
log_xh(:,1) = x_hat;
log_P(:,1)  = diag(P);

for k = 2:N
    z          = blocks.sensors(traj.pose(:,k), traj.pose(:,k-1), k-1, dt, p);
    [x_hat, P] = blocks.estimator(x_hat, P, z, dt, p);
    log_xh(:,k) = x_hat;
    log_P(:,k)  = diag(P);
end

%% Errors
e_xy  = log_xh(1:2,:) - traj.pose(1:2,:);
e_pos = vecnorm(e_xy, 2, 1);
e_th  = wrap_pi(log_xh(3,:) - traj.pose(3,:));

% Lock (3.2.2.6): first time after which position error stays in budget
outside = find(e_pos > p.budget.nav, 1, 'last');
if isempty(outside),     t_lock = 0;
elseif outside == N,     t_lock = inf;
else,                    t_lock = traj.t(outside + 1);
end

% Consistency: does the filter's own covariance cover the real error?
sig_xy = sqrt(log_P(1,:) + log_P(2,:));       % horizontal 1-sigma (approx)
in2sig = mean(abs(e_xy(1,:)) <= 2*sqrt(log_P(1,:)) & ...
              abs(e_xy(2,:)) <= 2*sqrt(log_P(2,:)) & ...
              abs(e_th)      <= 2*sqrt(log_P(3,:)));

%% Results
res.traj      = traj;
res.t         = traj.t;
res.x_true    = traj.pose;
res.x_hat     = log_xh;
res.P_diag    = log_P;
res.e_xy      = e_xy;
res.e_pos     = e_pos;
res.e_th      = e_th;
res.sig_xy    = sig_xy;
res.pos_rms   = sqrt(mean(e_pos.^2));
res.pos_95    = prctile(e_pos, 95);
res.pos_max   = max(e_pos);
res.pos_end   = e_pos(end);
res.th_rms    = sqrt(mean(e_th.^2));
res.th_95     = prctile(abs(e_th), 95);
res.t_lock    = t_lock;
res.in_2sigma = in2sig;      % ~0.87 expected for 3 independent states
end
