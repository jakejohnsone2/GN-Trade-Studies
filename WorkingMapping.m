clear
close all
clc
 
% Authored by Jacob Johnson, Claude added parameter tweaking and visual
% graphing improvements.
%% ============================ Parameters ================================
 
Total_size = 5;             % (m) Size of search area
n = 50;                     % Total intervals for mapping (change for more complexity)
 
Obstacle_fraction = 0.30;   % Fraction of the map to block (scales with n)
 
% Setting the start and end position
Start_Position = [0,0];
End_Position   = [4.5, 4.2];
 
% Calculating the vertical and horizontal distance between nodes
Spacing = Total_size/n;
 
% Calculating the total number of nodes per row/column
% (n intervals means n+1 nodes -- you need a node at both ends)
Nodes = n+1;
 
 
%% ========================== Creating Grid ===============================
 
% Creating a mesh grid that represents the nodes in the graph
[X, Y] = meshgrid(0:Spacing:Total_size, 0:Spacing:Total_size);
 
% Flattening our matrices for our future graph
x = X(:);
y = Y(:);
total_nodes = length(x);
 
% Initializing our s and t vectors for the for loop
s = [];
t = [];
weights = [];
 
% For loop that connects each of our nodes
for yval = 1:Nodes
    for xval = 1:Nodes
        % Creating an id for our current node
        Curent_Node = (yval-1)*Nodes + xval;
 
        % Connecting our node with the neighbor to the right
        if xval < Nodes
            neighbor_right_id = Curent_Node + 1;
 
            s = [s; Curent_Node];
            t = [t; neighbor_right_id];
            weights = [weights; Spacing];
        end
 
        % Connecting our node with the neighbor above
        if yval < Nodes
            neighbor_above_id = yval*Nodes + xval;
 
            s = [s; Curent_Node];
            t = [t; neighbor_above_id];
            weights = [weights; Spacing];
        end
 
        % Connecting our node with the upper right diagonal
        if yval < Nodes && xval < Nodes
            neighbor_ur_diagonal_id = yval*Nodes + xval + 1;
 
            s = [s; Curent_Node];
            t = [t; neighbor_ur_diagonal_id];
            weights = [weights; Spacing*sqrt(2)];
        end
 
        % Connecting our node with the upper left diagonal
        if yval < Nodes && xval > 1
            neighbor_ul_diagonal_id = yval*Nodes + xval - 1;
 
            s = [s; Curent_Node];
            t = [t; neighbor_ul_diagonal_id];
            weights = [weights; Spacing*sqrt(2)];
        end
 
    end
end
 
 
%% ====================== Start, Goal, Obstacles ==========================
 
% Converting an (x,y) position into a node ID.
% NOTE: meshgrid puts x along the COLUMNS and (:) flattens column-major,
% so the x term is the one multiplied by Nodes. (The original version had
% px and py swapped -- harmless on a square grid, wrong on any other.)
pos2id = @(px,py) round(px/Spacing)*Nodes + round(py/Spacing) + 1;
 
start_id = pos2id(Start_Position(1), Start_Position(2));
goal_id  = pos2id(End_Position(1),   End_Position(2));
 
% CHANGE 1: obstacle count now scales with the grid instead of being
% hardcoded, so difficulty stays comparable when n changes.
Obstacle_number = round(Obstacle_fraction * total_nodes);
 
% CHANGE 2: randperm draws WITHOUT replacement, so we get exactly
% Obstacle_number distinct nodes (randi repeats and undershoots badly).
rng('default')
blocked = randperm(total_nodes, Obstacle_number).';
 
% Making sure the start and end aren't treated as obstacles
blocked(blocked == start_id | blocked == goal_id) = [];
 
% Finding the index of edges where either endpoint is blocked
bad = ismember(s, blocked) | ismember(t, blocked);
 
% Removing the edges of blocked nodes
s(bad) = [];  t(bad) = [];  weights(bad) = [];
 
% CHANGE 3: the 4th argument pins the node count. Without it, graph()
% infers the node count from the largest ID still present in s/t -- so if
% the highest-numbered node gets blocked, the graph silently shrinks and
% node k no longer corresponds to (x(k), y(k)).
graphObject = graph(s, t, weights, total_nodes);
 
 
%% ====================== Sanity Checks on the Map ========================
 
% Is the goal reachable at all? conncomp labels each node with its
% connected-component number; matching labels means a path exists.
bins = conncomp(graphObject);
reachable = (bins(start_id) == bins(goal_id));
 
fprintf('Grid: %d x %d = %d nodes\n', Nodes, Nodes, total_nodes);
fprintf('Obstacles placed: %d (%.0f%% of map)\n', ...
        numel(blocked), 100*numel(blocked)/total_nodes);
fprintf('Start node degree: %d   Goal node degree: %d\n', ...
        numel(neighbors(graphObject, start_id)), ...
        numel(neighbors(graphObject, goal_id)));
fprintf('Start and goal connected: %d\n', reachable);
 
if ~reachable
    warning(['Goal is not reachable with this obstacle field. ' ...
             'Lower Obstacle_fraction or change the rng seed.']);
end
 
 
%% ============================= Plotting =================================
% CHANGE 4: plotting now happens AFTER the edges are pruned, so the figure
% only draws edges that actually exist. Previously the picture still showed
% edges passing straight through the obstacles.
 
figure
plotObject = plot(graphObject, 'XData', x, 'YData', y);
 
% Styling the map
plotObject.Marker     = 'o';
plotObject.MarkerSize = 0.5;
plotObject.NodeColor  = 'b';
plotObject.LineWidth  = 0.5;
plotObject.EdgeColor  = 'k';
axis equal;
grid on;
xlabel('X Position in Search Zone (m)')
ylabel('Y Position in Search Zone (m)')
title('Gridded Map of Nodes with Obstacles')
 
hold on
 
% Highlighting the blocked points
highlight(plotObject, blocked, 'NodeColor', 'k', 'MarkerSize', 2);
 
% Marking start and goal
plot(x(start_id), y(start_id), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(x(goal_id),  y(goal_id),  'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
 
hold off
 
