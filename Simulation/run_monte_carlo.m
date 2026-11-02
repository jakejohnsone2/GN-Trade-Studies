%% RUN_MONTE_CARLO Verify the error budget over many missions.
%   Part 1: N seeds, Gaussian marker, each seed run twice (controller sees
%           truth, then x_hat). The gap between them is the coupling.
%   Part 2: marker at the 15 cm hard bound, swept around the ring (GN.2a).
clear; close all; clc;
setup_paths();

N        = 200;     % missions per feedback mode
n_ring   = 16;      % directions for the worst-case sweep
base     = params();
blocks   = default_blocks();
modes    = {'truth', 'estimate'};

%% Part 1: random marker error, both feedback modes
stats = struct();
for m = 1:numel(modes)
    p = base;
    p.sim.feedback = modes{m};
    s = init_stats(N);
    for i = 1:N
        p.sim.seed = i;
        s = record(s, i, simulate_mission(p, blocks));
    end
    stats.(modes{m}) = s;
end

%% Part 2: worst-case marker ring, real closed loop
p = base;
p.marker.mode  = 'worst_ring';
p.sim.feedback = 'estimate';
ring = init_stats(n_ring);
for i = 1:n_ring
    p.sim.seed         = i;
    p.marker.ring_angle = 2*pi*(i-1)/n_ring;
    ring = record(ring, i, simulate_mission(p, blocks));
end

%% Report (95th percentile radii, cm)
T = stats.truth;  E = stats.estimate;  b = base.budget;
fprintf('\n%d missions per mode, Gaussian marker (95th percentile, cm)\n', N);
fprintf('%-34s %8s %8s %6s\n', 'Check', 'Result', 'Budget', 'Pass');
row('e_stop, perfect knowledge (IF-DC.1)', pct95(T.e_stop), b.stop);
row('e_nav at the stop (GN.1)',            pct95(E.est),    b.nav);
row('Rover error, perfect knowledge',      pct95(T.rover),  b.rover);
row('Rover error, closed loop (GN.0)',     pct95(E.rover),  b.rover);
row('Total miss, closed loop (SYS.4)',     pct95(E.total),  b.total);
fprintf('Coupling (closed loop minus perfect knowledge): %+.1f cm\n', ...
        100*(pct95(E.rover) - pct95(T.rover)));
fprintf('RSS prediction sqrt(e_nav^2 + e_stop^2): %.1f cm\n', ...
        100*hypot(pct95(E.est), pct95(T.e_stop)));
fprintf('Success rate within 30 cm: %.1f %%   timeouts: %d\n', ...
        100*mean(E.success), sum(E.timed_out));

fprintf('\nWorst-case marker ring, %d directions at %.0f cm\n', n_ring, 100*base.marker.bound);
fprintf('Max total miss: %.1f cm   all within 30 cm: %d\n', ...
        100*max(ring.total), all(ring.success));

%% Plots
figure('Name', 'Monte Carlo', 'Color', 'w');
edges = 0:0.5:20;
histogram(100*T.rover, edges, 'DisplayName', 'Perfect knowledge'); hold on;
histogram(100*E.rover, edges, 'DisplayName', 'Closed loop (x\_hat)');
xline(100*b.rover, 'k--', 'GN.0 15 cm', 'HandleVisibility', 'off');
xlabel('Rover error at stop (cm)'); ylabel('Missions'); grid on;
legend; title('Rover error, true stop vs goal waypoint');

%% ------------------------------------------------------------------------
function s = init_stats(n)
z = nan(n,1);
s = struct('e_marker', z, 'e_stop', z, 'e_nav', z, 'est', z, 'rover', z, ...
           'total', z, 'success', false(n,1), 'timed_out', false(n,1));
end

function s = record(s, i, res)
e = res.err;
s.e_marker(i)  = norm(e.e_marker);
s.e_stop(i)    = norm(e.e_stop);
s.e_nav(i)     = norm(e.e_nav);
s.est(i)       = norm(e.est);
s.rover(i)     = norm(e.rover);
s.total(i)     = norm(e.total);
s.success(i)   = res.success;
s.timed_out(i) = res.timed_out;
end

function v = pct95(x)
x = sort(x(~isnan(x)));
v = x(ceil(0.95*numel(x)));
end

function row(name, val, budget)
pass = 'yes'; if val > budget, pass = 'NO'; end
fprintf('%-34s %8.1f %8.1f %6s\n', name, 100*val, 100*budget, pass);
end
