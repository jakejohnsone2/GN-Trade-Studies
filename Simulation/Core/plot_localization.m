function plot_localization(res, p)
%PLOT_LOCALIZATION Path, position error and heading error with 2-sigma bounds.
figure('Name', ['Localization: ' res.traj.name], 'Color', 'w');

subplot(2,2,[1 3]); hold on; axis equal; grid on; box on;
for i = 1:size(p.map.obstacles,1)
    o = p.map.obstacles(i,:);
    rectangle('Position', [o(1)-o(3), o(2)-o(3), 2*o(3), 2*o(3)], ...
              'Curvature', [1 1], 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', 'none');
end
h1 = plot(res.x_true(1,:), res.x_true(2,:), 'k-',  'LineWidth', 1.5);
h2 = plot(res.x_hat(1,:),  res.x_hat(2,:),  'b--', 'LineWidth', 1.0);
xlim([0 p.map.size]); ylim([0 p.map.size]);
xlabel('East (m)'); ylabel('North (m)');
legend([h1 h2], 'truth', 'estimate', 'Location', 'best');
title(sprintf('"%s"  95 Percent: %.1f cm  (budget %.0f cm)', ...
      res.traj.name, 100*res.pos_95, 100*p.budget.nav));

subplot(2,2,2); hold on; grid on; box on;
plot(res.t, 100*res.e_pos, 'b');
plot(res.t, 100*2*res.sig_xy, 'r--');
yline(100*p.budget.nav, 'k:', '3.2.2.4');
xlabel('t (s)'); ylabel('position error (cm)');
legend('error', '2\sigma (filter)', 'Location', 'best');

subplot(2,2,4); hold on; grid on; box on;
s = rad2deg(2*sqrt(res.P_diag(3,:)));
plot(res.t, rad2deg(res.e_th), 'b');
plot(res.t, s, 'r--'); plot(res.t, -s, 'r--');
xlabel('t (s)'); ylabel('heading error (deg)');
end
