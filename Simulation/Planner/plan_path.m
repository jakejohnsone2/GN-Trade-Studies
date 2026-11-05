function path = plan_path(map, start_xy, goal_xy, p)
%PLAN_PATH Plan a collision-free route with 8-connected A* on an occupancy grid.
%   The map stores circular obstacles one per row as [x y radius]. Each
%   circle is expanded by map.clearance for rover safety before rasterizing.
%   Grid rows represent North/y and columns represent East/x.
%
%   path.wp          Nx2 waypoints in [x y] meters; first is start and last
%                    is the exact goal supplied to this function.
%   path.final_ok    true when the final leg is at least p.guid.L_f meters.
%   path.cost        A* grid cost in meters.
%
%   Diagonal moves are rejected if either adjacent orthogonal cell is blocked,
%   preventing corner cutting. The cell-by-cell route is shortened by removing
%   points visible along a clear straight segment. Exact endpoints are kept.

% Normalize coordinates to row vectors so they match the waypoint format.
start_xy = start_xy(:)';
goal_xy  = goal_xy(:)';

% Create uniformly spaced grid coordinates over the square map. A* cost is
% measured in grid steps, so h converts that cost back to meters at the end.
h        = map.grid_spacing;
g        = 0:h:map.size;
n        = numel(g);

% This grid setup requires 2-D positions and a grid that reaches map.size
% exactly; otherwise the last cell would have a different width.
if numel(start_xy) ~= 2 || numel(goal_xy) ~= 2
    error('plan_path:coordinates', 'Start and goal must each be [x y].');
end
if n < 2 || abs(g(end) - map.size) > 10*eps(max(1,map.size))
    error('plan_path:grid', 'map.size must be an integer multiple of map.grid_spacing.');
end

% Build each cell's [x,y] coordinate, then mark it blocked if it falls within
% an obstacle radius plus the required clearance.
[X, Y] = meshgrid(g, g);
blocked = ~is_free([X(:), Y(:)], map);
blocked = reshape(blocked, n, n);

% A* uses integer cell indices. Snap continuous start/goal positions to the
% nearest free cells; the original coordinates are restored in the output.
startRC = nearest_free_rc(blocked, start_xy, g);
goalRC  = nearest_free_rc(blocked, goal_xy, g);
[pathRC, totalCost, found] = astar_grid(blocked, startRC, goalRC);
if ~found
    error('plan_path:noPath', 'No collision-free path to the goal.');
end

% Convert [row,column] cells to [x,y] meters. Columns map to x; rows to y.
grid_xy = [g(pathRC(:,2))', g(pathRC(:,1))'];
pts = [start_xy; grid_xy; goal_xy];

% Greedily connect each retained waypoint to the farthest later point with
% clear line of sight. This removes grid zigzags and reduces controller turns.
wp = pts(1,:);
i = 1;
while i < size(pts,1)
    j = size(pts,1);
    while j > i+1 && ~segment_free(pts(i,:), pts(j,:), map)
        j = j - 1;
    end
    if norm(pts(j,:) - wp(end,:)) > 10*eps(max(1,map.size))
        wp(end+1,:) = pts(j,:); %#ok<AGROW>
    end
    i = j;
end
if size(wp,1) == 1
    wp = [wp; goal_xy];
end

% Package waypoints for the controller. Cost is the raw A* grid cost in
% steps times grid spacing, expressed in meters; simplification does not
% change this reported grid-route cost.
path.wp = wp;
path.cost = totalCost * h;
% Check the guidance requirement for the minimum straight final approach.
path.final_ok = size(wp,1) >= 2 && ...
    norm(wp(end,:) - wp(end-1,:)) >= p.guid.L_f;
if ~path.final_ok
    warning('plan_path:finalLeg', ...
        'Final leg is %.2f m, shorter than L_f = %.2f m (3.2.1.1).', ...
        norm(wp(end,:) - wp(max(1,end-1),:)), p.guid.L_f);
end
end

function [pathRC, totalCost, found] = astar_grid(blocked, startRC, goalRC)
%ASTAR_GRID A* implementation adapted from the supplied astarGrid function.
% gScore stores cost from the start, fScore adds a lower-bound estimate to
% the goal, cameFrom stores predecessors, and open/closed track search state.
[H, W] = size(blocked);
pathRC = zeros(0,2);
totalCost = inf;
found = false;
sr = startRC(1); sc = startRC(2);
gr = goalRC(1); gc = goalRC(2);

% The snapped start and goal may be the same grid cell.
if sr == gr && sc == gc
    pathRC = [sr sc];
    totalCost = 0;
    found = true;
    return
end

% Use MATLAB linear indices so each search array has one entry per grid cell.
N = H*W;
startIdx = sub2ind([H W], sr, sc);
goalIdx = sub2ind([H W], gr, gc);
gScore = inf(N,1);
fScore = inf(N,1);
cameFrom = zeros(N,1,'uint32');
closed = false(N,1);
open = false(N,1);
gScore(startIdx) = 0;
fScore(startIdx) = octile(sr,sc,gr,gc);
open(startIdx) = true;

% Four cardinal directions and four diagonals; diagonal distance is sqrt(2).
dRC = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
moveCost = [1; 1; 1; 1; sqrt(2); sqrt(2); sqrt(2); sqrt(2)];
% Search until the goal is reached or no accessible frontier remains.
while any(open)
    % Expand the frontier cell with the smallest estimated total cost.
    openIdx = find(open);
    [~,q] = min(fScore(openIdx));
    current = openIdx(q);
    % Follow predecessor links back to the start, then reverse the sequence.
    if current == goalIdx
        reverse = zeros(N,1);
        count = 1;
        reverse(count) = current;
        while current ~= startIdx
            current = double(cameFrom(current));
            if current == 0
                return
            end
            count = count + 1;
            reverse(count) = current;
        end
        indices = flipud(reverse(1:count));
        [rr,cc] = ind2sub([H W],indices);
        pathRC = [rr(:),cc(:)];
        totalCost = gScore(goalIdx);
        found = true;
        return
    end

    % This cell has been expanded; it is no longer an active candidate.
    open(current) = false;
    closed(current) = true;
    [r,c] = ind2sub([H W],current);
    for j = 1:8
        nr = r + dRC(j,1);
        nc = c + dRC(j,2);
        % Reject cells outside the grid and cells occupied by obstacles.
        if nr < 1 || nr > H || nc < 1 || nc > W || blocked(nr,nc)
            continue
        end
        % Disallow diagonal passage between blocked cardinal neighbors.
        if j >= 5 && (blocked(r,nc) || blocked(nr,c))
            continue
        end
        neighbor = sub2ind([H W],nr,nc);
        if closed(neighbor)
            continue
        end
        % Record a better route to this neighbor and update its A* scores.
        tentativeG = gScore(current) + moveCost(j);
        if ~open(neighbor) || tentativeG < gScore(neighbor)
            cameFrom(neighbor) = uint32(current);
            gScore(neighbor) = tentativeG;
            fScore(neighbor) = tentativeG + octile(nr,nc,gr,gc);
            open(neighbor) = true;
        end
    end
end
end

function h = octile(r,c,gr,gc)
%OCTILE Admissible 8-way distance estimate for A* (no obstacles assumed).
dr = abs(r-gr);
dc = abs(c-gc);
h = max(dr,dc) + (sqrt(2)-1)*min(dr,dc);
end

function rc = nearest_free_rc(blocked, q, g)
%NEAREST_FREE_RC Find the free grid cell whose physical location is nearest q.
[X,Y] = meshgrid(g,g);
d = hypot(X-q(1), Y-q(2));
d(blocked) = inf;
[best,idx] = min(d(:));
if isinf(best)
    error('plan_path:noFreeCell', 'The occupancy grid contains no free cells.');
end
[r,c] = ind2sub(size(blocked),idx);
rc = [r,c];
end

function free = is_free(xy,map)
%IS_FREE Return one boolean per point; points within expanded obstacles fail.
free = true(size(xy,1),1);
for k = 1:size(map.obstacles,1)
    center = map.obstacles(k,1:2);
    radius = map.obstacles(k,3) + map.clearance;
    free = free & hypot(xy(:,1)-center(1),xy(:,2)-center(2)) > radius;
end
end

function ok = segment_free(a,b,map)
%SEGMENT_FREE Sample the straight line from a to b and require all samples free.
% Sampling at half grid spacing makes this check finer than the occupancy grid.
nPts = max(2,ceil(norm(b-a)/(map.grid_spacing/2)));
lambda = linspace(0,1,nPts)';
ok = all(is_free(a + lambda.*(b-a),map));
end
