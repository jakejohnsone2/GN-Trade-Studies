%% Localization Studies
% Author: Xander Lotito
%
% Sensors + estimator only, along scripted trajectories (no D&C model).
% Change sensor noise in params.m (p.sens.*) or swap blocks.sensors /
% blocks.estimator, then rerun to compare.
clc
clear
close all
setup_paths()

p      = params();
blocks = default_blocks();

%% Single run, one trajectory
traj = scripted_trajectory('mission', p);
res  = simulate_localization(p, blocks, traj);
plot_localization(res, p);

%% Monte Carlo over every trajectory
names = {'stationary', 'straight', 'square', 'arc', 'figure8', 'mission'};
n_runs = 50;

fprintf('\n%-11s %9s %9s %9s %9s %8s %8s\n', 'trajectory', 'RMS(cm)', ...
        '95%(cm)', 'max(cm)', 'hdg95(d)', 'lock(s)', 'in2sig');
for i = 1:numel(names)
    traj = scripted_trajectory(names{i}, p);
    e_all = []; th_all = []; t_lock = nan(1,n_runs); cov = nan(1,n_runs);
    for r = 1:n_runs
        p.sim.seed = r;
        res = simulate_localization(p, blocks, traj);
        e_all  = [e_all, res.e_pos];      %#ok<AGROW>
        th_all = [th_all, res.e_th];      %#ok<AGROW>
        t_lock(r) = res.t_lock;
        cov(r)    = res.in_2sigma;
    end
    fprintf('%-11s %9.2f %9.2f %9.2f %9.2f %8.2f %8.2f\n', names{i}, ...
            100*sqrt(mean(e_all.^2)), 100*prctile(e_all,95), 100*max(e_all), ...
            rad2deg(prctile(abs(th_all),95)), max(t_lock), mean(cov));
end
fprintf('Budget 3.2.2.4: %.0f cm (95%%). Lock 3.2.2.6: 60 s.\n', 100*p.budget.nav);
