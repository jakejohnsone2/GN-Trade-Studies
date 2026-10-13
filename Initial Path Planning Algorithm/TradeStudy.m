clear
close all
clc
% Path planning trade study: Depth First Search vs Dijkstra vs A*
% on a shared 8-connected grid with clustered obstacles.
%
% Authored by Jacob, Ignacio, and Xander. Claude assisted with parameter handling,
% obstacle clustering and plotting.

%% ============================ PARAMETERS ================================
% Every knob for the whole study lives here. Nothing below this block
% needs editing to change the map or the run.

% --- Search area ---
Total_size     = 5;             % (m) Size of the square search area
n              = 100;           % Intervals per side (higher = finer grid)

% --- Start and goal ---
Start_Position = [0, 0];        % (m)
End_Position   = [4.5, 4.2];    % (m)

% --- Obstacle field ---
Obstacle_fraction = 0.40;       % Fraction of the map to block
Clump_size        = 0.01;       % (m) characteristic obstacle radius
Clear_radius      = 0.30;       % (m) clearance kept around start & goal
max_attempts      = 20;         % reseeds allowed to find a solvable map

% --- Output ---
save_mat   = true;              % write MapData.mat for other scripts
fig_name   = 'PathPlanningTradeStudy';
BLOCKSIZE = 7;

% --- Derived (do not edit) ---
Spacing = Total_size/n;         % (m) distance between adjacent nodes
Nodes   = n + 1;                % nodes per row/column (n gaps = n+1 posts)


%% ========================== Creating Grid ===============================
% Node coordinates. meshgrid puts x along the columns; (:) flattens
% column-major, so node k sits at (x(k), y(k)).

[X, Y] = meshgrid(0:Spacing:Total_size, 0:Spacing:Total_size);
x = X(:);
y = Y(:);
total_nodes = length(x);

% Edge list: row k says "an edge joins node s(k) to node t(k), and it is
% weights(k) metres long".
s = [];
t = [];
weights = [];

% Each node only emits edges pointing right, up, and the two upward
% diagonals. graph() is undirected, so the reverse direction comes free
% when the neighbour takes its own turn -- that is how all 8 connections
% get made without duplicates.
for yval = 1:Nodes
    for xval = 1:Nodes

        Curent_Node = (yval-1)*Nodes + xval;

        if xval < Nodes                                  % right
            s = [s; Curent_Node];
            t = [t; Curent_Node + 1];
            weights = [weights; Spacing];
        end

        if yval < Nodes                                  % above
            s = [s; Curent_Node];
            t = [t; yval*Nodes + xval];
            weights = [weights; Spacing];
        end

        if yval < Nodes && xval < Nodes                  % upper right
            s = [s; Curent_Node];
            t = [t; yval*Nodes + xval + 1];
            weights = [weights; Spacing*sqrt(2)];
        end

        if yval < Nodes && xval > 1                      % upper left
            s = [s; Curent_Node];
            t = [t; yval*Nodes + xval - 1];
            weights = [weights; Spacing*sqrt(2)];
        end

    end
end


%% ============================ Start / Goal ==============================
% x is the term multiplied by Nodes, matching the column-major flattening.

pos2id = @(px,py) round(px/Spacing)*Nodes + round(py/Spacing) + 1;

start_id = pos2id(Start_Position(1), Start_Position(2));
goal_id  = pos2id(End_Position(1),   End_Position(2));


%% ======================= Clustered Obstacles ============================
% Independent random nodes give confetti. Real obstacles are correlated:
% if one spot is blocked its neighbour probably is too. So start from
% white noise, blur it (neighbouring values now move together), and keep
% the highest values. The blur width sets clump size; the cutoff sets
% coverage. The two knobs are independent.

Obstacle_number = round(Obstacle_fraction * total_nodes);

% Gaussian kernel. sigma converts clump size from metres to nodes, so
% obstacles keep the same physical size when n changes.
sigma  = max(Clump_size / Spacing, 0.5);
radius = ceil(3*sigma);
[kx, ky] = meshgrid(-radius:radius, -radius:radius);
K = exp(-(kx.^2 + ky.^2) / (2*sigma^2));
K = K / sum(K(:));

% Clearance bubbles so start and goal can't be walled in
d_start = hypot(x - x(start_id), y - y(start_id));
d_goal  = hypot(x - x(goal_id),  y - y(goal_id));

% Untouched copy of the full lattice, so each attempt prunes from the
% original rather than from an already-pruned edge list
s_full = s;  t_full = t;  w_full = weights;

% Clumped obstacles seal off the map far more often than scattered ones,
% because blobs merge into barriers. Generate, test, reseed if needed.
reachable = false;

for attempt = 1:max_attempts

    rng(attempt);

    noise = randn(Nodes, Nodes);                    % white noise
    field = conv2(noise, K, 'same');                % blur -> clumping

    % Undo conv2's zero-padding attenuation so density stays uniform
    % instead of thinning out at the borders
    field = field ./ sqrt(conv2(ones(Nodes), K.^2, 'same'));

    % Cut at a fixed position in the sorted values -> exact obstacle
    % count, and because the field is smooth they come out as patches.
    % field(:) flattens column-major like x and y, so position k is node k.
    vals      = sort(field(:), 'descend');
    threshold = vals(min(Obstacle_number, numel(vals)));
    blocked   = find(field(:) >= threshold);

    blocked(d_start(blocked) < Clear_radius | d_goal(blocked) < Clear_radius) = [];
    blocked(blocked == start_id | blocked == goal_id) = [];

    % Drop every edge with a blocked endpoint, then rebuild. The 4th
    % argument pins the node count so node k stays at (x(k), y(k)).
    bad     = ismember(s_full, blocked) | ismember(t_full, blocked);
    s       = s_full(~bad);
    t       = t_full(~bad);
    weights = w_full(~bad);

    graphObject = graph(s, t, weights, total_nodes);

    bins      = conncomp(graphObject);
    reachable = (bins(start_id) == bins(goal_id));

    if reachable
        break
    end
end

fprintf('Grid: %d x %d = %d nodes, %d edges\n', ...
        Nodes, Nodes, total_nodes, numedges(graphObject));
fprintf('Clump size: %.2f m (sigma = %.1f nodes)\n', Clump_size, sigma);
fprintf('Obstacles: %d (%.1f%% of map), found in %d attempt(s)\n', ...
        numel(blocked), 100*numel(blocked)/total_nodes, attempt);
fprintf('Start degree: %d   Goal degree: %d   Connected: %d\n\n', ...
        numel(neighbors(graphObject, start_id)), ...
        numel(neighbors(graphObject, goal_id)), reachable);

if ~reachable
    error(['No solvable map found in %d attempts. Lower ' ...
           'Obstacle_fraction or reduce Clump_size.'], max_attempts);
end


%% ========================= Depth First Search ===========================
% dfsearch returns discovery edges in the order DFS found them. Turning
% those into parent pointers gives the DFS tree path to the goal.

tic
events = dfsearch(graphObject, start_id, 'edgetonew');

parent = zeros(total_nodes,1);
for k = 1:size(events,1)
    parent(events(k,2)) = events(k,1);
end

if goal_id ~= start_id && parent(goal_id) == 0
    error('Goal is not reachable from the start node.');
end

path = goal_id;
while path(1) ~= start_id
    path = [parent(path(1)); path];
end
time_dfs = toc;


%% ============================= Dijkstra =================================

tic
[path_dijkstra, dijkstra_cost] = shortestpath(graphObject, start_id, goal_id);
time_dijkstra = toc;


%% ============================= A* Search ================================
% Straight-line distance to the goal. Edge weights are true Euclidean
% lengths, so this never overestimates -> admissible -> optimal result.

h = @(node_id) hypot(x(goal_id) - x(node_id), y(goal_id) - y(node_id));

g_score   = inf(total_nodes, 1);
f_score   = inf(total_nodes, 1);
came_from = zeros(total_nodes, 1);
in_open   = false(total_nodes, 1);
in_closed = false(total_nodes, 1);

g_score(start_id) = 0;
f_score(start_id) = h(start_id);
in_open(start_id) = true;

found = false;
tic

while any(in_open)

    % Lowest f among open nodes. temp_f stays full length so position k
    % is still node k.
    temp_f = f_score;
    temp_f(~in_open) = inf;
    [~, current] = min(temp_f);

    if current == goal_id
        found = true;
        break
    end

    in_open(current)   = false;
    in_closed(current) = true;

    nbrs = neighbors(graphObject, current);

    for k = 1:numel(nbrs)
        nb = nbrs(k);

        if in_closed(nb)
            continue
        end

        eid       = findedge(graphObject, current, nb);
        edge_cost = graphObject.Edges.Weight(eid);
        temp_g    = g_score(current) + edge_cost;

        if temp_g < g_score(nb)
            came_from(nb) = current;
            g_score(nb)   = temp_g;
            f_score(nb)   = g_score(nb) + h(nb);
            in_open(nb)   = true;
        end
    end
end

time_astar = toc;

if ~found
    error('A* failed to reach the goal.');
end

path_A = goal_id;
while path_A(1) ~= start_id
    predecessor = came_from(path_A(1));
    if predecessor == 0
        error('Failed to reconstruct a valid path.');
    end
    path_A = [predecessor; path_A];
end


%% ============================== Figure ==================================

figure
hold on

% Grey lattice so the coloured paths stand out
plotObject = plot(graphObject, 'XData', x, 'YData', y);
plotObject.Marker     = 'o';
plotObject.MarkerSize = 0.5;
plotObject.NodeColor  = [0.65 0.65 0.65];
plotObject.LineWidth  = 0.1;
plotObject.EdgeColor  = [0.85 0.85 0.85];
plotObject.NodeLabel  = {};

% Obstacles as a real plot, not highlight(), so the legend gets a handle
hBlocked = plot(x(blocked), y(blocked), 'ks', ...
                'MarkerSize', BLOCKSIZE, 'MarkerFaceColor', 'k', 'LineStyle', 'none');

% Each path is its own line object. highlight() recolours the shared
% edges of plotObject, so whichever ran last would erase the others
% wherever two paths overlap. Thickest first, thinnest on top.
hDFS = plot(x(path), y(path), '-', 'Color', [0.925 0.251 0.478 0.35], 'LineWidth', 3.0);
hDij = plot(x(path_dijkstra), y(path_dijkstra), '-',  'Color', '#E65100', 'LineWidth', 3.0);
hAst = plot(x(path_A),        y(path_A),        '--', 'Color', '#3949AB', 'LineWidth', 1.8);

hLine  = plot([Start_Position(1) End_Position(1)], ...
              [Start_Position(2) End_Position(2)], ...
              'g--', 'LineWidth', 1.8);

hStart = plot(x(start_id), y(start_id), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
hGoal  = plot(x(goal_id),  y(goal_id),  'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');

hold off

axis equal
pad = Spacing;
xlim([-pad Total_size+pad])
ylim([-pad Total_size+pad])
grid on
xlabel('X Position in Search Zone (m)')
ylabel('Y Position in Search Zone (m)')
title('Path Planning Trade Study')

legend([hDFS hDij hAst hLine hBlocked hStart hGoal], ...
       {'Depth First', 'Dijkstra', 'A*', 'Straight Line', ...
        'Obstacles', 'Start', 'Goal'}, ...
       'Location', 'northwest');

% print AFTER the legend, or the saved image won't have one
print(fig_name, '-dpng', '-r200')


%% ============================== Numbers =================================

% Path length in metres = sum of the edge weights along the route
pathlen = @(p) sum(graphObject.Edges.Weight( ...
                   findedge(graphObject, p(1:end-1), p(2:end))));

straight_line = hypot(End_Position(1)-Start_Position(1), ...
                      End_Position(2)-Start_Position(2));

fprintf('---- Trade Study ----\n');
fprintf('%-14s %10s %8s %10s\n', 'Algorithm', 'Cost (m)', 'Nodes', 'Time (s)');
fprintf('%-14s %10.4f %8d %10.4f\n', 'Depth First', pathlen(path),      numel(path),          time_dfs);
fprintf('%-14s %10.4f %8d %10.4f\n', 'Dijkstra',    dijkstra_cost,      numel(path_dijkstra), time_dijkstra);
fprintf('%-14s %10.4f %8d %10.4f\n', 'A*',          g_score(goal_id),   numel(path_A),        time_astar);
fprintf('%-14s %10.4f\n',            'Straight line', straight_line);
fprintf('\nA* nodes expanded: %d of %d (%.1f%% of the map)\n', ...
        nnz(in_closed), total_nodes, 100*nnz(in_closed)/total_nodes);

if abs(g_score(goal_id) - dijkstra_cost) < 1e-9
    fprintf('A* matches Dijkstra -- optimal.\n');
else
    warning('A* cost differs from Dijkstra by %.3e -- check the heuristic.', ...
            abs(g_score(goal_id) - dijkstra_cost));
end

if save_mat
    save('MapData.mat', 'graphObject', 'x', 'y', 'start_id', 'goal_id', ...
         'blocked', 'total_nodes');
end
