function path = plan_path(map, start_xy, goal_xy, p)
%PLAN_PATH Stand-in planner: grid shortest path, shortened to straight legs.
%   Owner: Planner. Replace with the A* from the trade study when ready.
%
%   path.wp          Nx2 waypoints (m). wp(1,:) is the start, wp(end,:) is
%                    the exact goal (not a grid node), so planning adds no
%                    error at the stop.
%   path.final_ok    true if the last leg is at least p.guid.L_f long (GN.3).
%
%   Legs are straight; the controller pivots in place at each waypoint.

start_xy = start_xy(:)';
goal_xy  = goal_xy(:)';
h        = map.grid_spacing;

%% Grid of free nodes
g        = 0:h:map.size;
n        = numel(g);
[X, Y]   = meshgrid(g, g);
free     = is_free([X(:), Y(:)], map);
idx      = reshape(1:n^2, n, n);

%% 8-connected edges between free neighbours
s = []; t = []; w = [];
offsets = [0 1; 1 0; 1 1; 1 -1];             % [row col] steps
for o = 1:size(offsets,1)
    dr = offsets(o,1); dc = offsets(o,2);
    rows = max(1,1-dr):min(n,n-dr);
    cols = max(1,1-dc):min(n,n-dc);
    a = idx(rows, cols);
    b = idx(rows+dr, cols+dc);
    keep = free(a) & free(b);
    s = [s; a(keep)];                        %#ok<AGROW>
    t = [t; b(keep)];                        %#ok<AGROW>
    w = [w; repmat(h*hypot(dr,dc), nnz(keep), 1)];  %#ok<AGROW>
end
G = graph(s, t, w, n^2);

%% Shortest path between the free nodes nearest start and goal
node_s = nearest_free([X(:), Y(:)], free, start_xy);
node_g = nearest_free([X(:), Y(:)], free, goal_xy);
nodes  = shortestpath(G, node_s, node_g);
if isempty(nodes)
    error('plan_path:noPath', 'No collision-free path to the goal.');
end
pts = [start_xy; X(nodes)', Y(nodes)'; goal_xy];

%% Shorten to straight legs with line-of-sight checks
wp = pts(1,:);
i  = 1;
while i < size(pts,1)
    j = size(pts,1);
    while j > i+1 && ~segment_free(pts(i,:), pts(j,:), map)
        j = j - 1;
    end
    wp(end+1,:) = pts(j,:);                  %#ok<AGROW>
    i = j;
end

path.wp       = wp;
path.final_ok = norm(wp(end,:) - wp(end-1,:)) >= p.guid.L_f;
if ~path.final_ok
    warning('plan_path:finalLeg', ...
        'Final leg is %.2f m, shorter than L_f = %.2f m (GN.3).', ...
        norm(wp(end,:) - wp(end-1,:)), p.guid.L_f);
end
end

%% ------------------------------------------------------------------------
function ok = is_free(xy, map)
ok = true(size(xy,1), 1);
for k = 1:size(map.obstacles,1)
    c  = map.obstacles(k,1:2);
    r  = map.obstacles(k,3) + map.clearance;
    ok = ok & hypot(xy(:,1)-c(1), xy(:,2)-c(2)) > r;
end
end

function node = nearest_free(xy, free, q)
d        = hypot(xy(:,1)-q(1), xy(:,2)-q(2));
d(~free) = inf;
[~, node] = min(d);
end

function ok = segment_free(a, b, map)
n_pts = max(2, ceil(norm(b-a) / (map.grid_spacing/2)));
lam   = linspace(0, 1, n_pts)';
ok    = all(is_free(a + lam.*(b-a), map));
end
