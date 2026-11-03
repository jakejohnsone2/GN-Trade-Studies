function plot_mission(res, p)
%PLOT_MISSION Map, planned path, true and estimated tracks, marker circles.

figure('Name', 'Mission', 'Color', 'w');
hold on; axis equal; grid on; box on;
xlim([0 p.map.size]); ylim([0 p.map.size]);

ang = linspace(0, 2*pi, 100);
circ = @(c, r) [c(1) + r*cos(ang); c(2) + r*sin(ang)];

for k = 1:size(p.map.obstacles,1)
    c = p.map.obstacles(k,1:2);
    o = circ(c, p.map.obstacles(k,3));
    m = circ(c, p.map.obstacles(k,3) + p.map.clearance);
    fill(o(1,:), o(2,:), [0.6 0.6 0.6], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(m(1,:), m(2,:), ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
end

plot(res.path.wp(:,1), res.path.wp(:,2), 'ko--', 'DisplayName', 'Planned path');
plot(res.x_true(1,:), res.x_true(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'True track');
plot(res.x_hat(1,:), res.x_hat(2,:), 'r-', 'DisplayName', 'Estimate');

s = circ(res.m_true, p.budget.total);
b = circ(res.m_true, p.marker.bound);
plot(s(1,:), s(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', '3.2 30 cm');
plot(b(1,:), b(2,:), 'm:', 'DisplayName', 'UAS 15 cm bound');
plot(res.m_true(1), res.m_true(2), 'gp', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Marker (true)');
plot(res.m_est(1), res.m_est(2), 'mx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Marker (UAS)');

xlabel('East (m)'); ylabel('North (m)');
title(sprintf('Miss %.1f cm  |  feedback: %s', 100*norm(res.err.total), res.feedback));
legend('Location', 'northwest');
end
