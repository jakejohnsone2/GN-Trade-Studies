%% RUN_MISSION One mission, start to marker, with a plot and the error split.
clear; close all; clc;
setup_paths();

p      = params();
blocks = default_blocks();
res    = simulate_mission(p, blocks);

e = res.err;
fprintf('Feedback: %s   time: %.1f s   timed out: %d\n', res.feedback, res.t(end), res.timed_out);
fprintf('  e_marker  %6.1f cm   (UAS bound   %4.1f)\n', 100*norm(e.e_marker), 100*p.budget.marker);
fprintf('  e_stop    %6.1f cm   (IF-DC.1     %4.1f)\n', 100*norm(e.e_stop),   100*p.budget.stop);
fprintf('  e_nav     %6.1f cm   (GN.1        %4.1f)\n', 100*norm(e.e_nav),    100*p.budget.nav);
fprintf('  rover     %6.1f cm   (GN.0        %4.1f)\n', 100*norm(e.rover),    100*p.budget.rover);
fprintf('  total     %6.1f cm   (SYS.4       %4.1f)  success: %d\n', ...
        100*norm(e.total), 100*p.budget.total, res.success);

plot_mission(res, p);
