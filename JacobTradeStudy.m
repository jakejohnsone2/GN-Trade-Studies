clear
close all
clc



%% Creating Grid

Total_size = 5; % (m) Size of search are
n = 50; % Total intervals for mapping (Change for more complexity)
x_nodes = n;
y_nodes = n;
Obstacle_number = 1900;

% Setting the start and end position
Start_Position = [0,0];
End_Position = [4.5, 4.2];


% Calculating the vertical and horizontal distance between nodes
Spacing = Total_size/n;

% Creating a mesh grid that represents the nodes in the graph
[X, Y] = meshgrid(0:Spacing:Total_size,0:Spacing:Total_size);

% Flattening our matrices for our future graph
x = X(:);
y = Y(:);
total_nodes = length(x);

% Initializing our s and t vectors for the for loop
s = [];
t = [];
weights = [];

% Calculating the total number of nodes per row/column
Nodes = n+1;

% For loop that connect each of our nodes
for yval = 1:Nodes
    for xval = 1:Nodes
        % Creating an id for our current node
        Curent_Node = (yval-1)*Nodes + xval;

        % Connecting our node with the neigbor to the right
        if xval < Nodes
            % Finding the the id of the neighbor to the right
            neighbor_right_id = Curent_Node + 1;  
            
            % Repopulating the s and t matrix
            s = [s; Curent_Node];
            t = [t; neighbor_right_id];

            % Calculating the edge weight (Edge Distance)
            weights = [weights; Spacing];
        end

        % Connecting our node with the neigbor above
        if yval < Nodes
            % Finding the the id of the neighbor to the right
            neighbor_above_id = yval*Nodes + xval;  
            
            % Repopulating the s and t matrix
            s = [s; Curent_Node];
            t = [t; neighbor_above_id];

            % Calculating the edge weight (Edge Distance)
            weights = [weights; Spacing];
        end

        % Connect our node with the upper right diagonal
        if yval < Nodes && xval < Nodes
            % Finding the the id of the neighbor to the right
            neighbor_ur_diagonal_id = yval*Nodes + xval + 1;  
            
            % Repopulating the s and t matrix
            s = [s; Curent_Node];
            t = [t; neighbor_ur_diagonal_id];
            
            % Calculating the edge weight (Edge Distance)
            weights = [weights; Spacing*sqrt(2)];
        end

        % Connect our node with the upper right diagonal
        if yval < Nodes && xval > 1
            % Finding the the id of the neighbor to the right
            neighbor_ur_diagonal_id = yval*Nodes + xval - 1;  
            
            % Repopulating the s and t matrix
            s = [s; Curent_Node];
            t = [t; neighbor_ur_diagonal_id];
            
            % Calculating the edge weight (Edge Distance)
            weights = [weights; Spacing*sqrt(2)];
        end

    end
end


% Creating a graphing object
graphObject = graph(s, t, weights);
hold on

pos2id = @(px,py) round(px/Spacing)*Nodes + round(py/Spacing) + 1;
 
start_id = pos2id(Start_Position(1), Start_Position(2));
goal_id  = pos2id(End_Position(1),   End_Position(2));

%% Depth First Search

% Creating a function that calulates the node number
pos2id = @(px,py) round(px/Spacing)*Nodes + round(py/Spacing) + 1;

% Calculating the node number of starting and end position
start_id = pos2id(Start_Position(1), Start_Position(2));
goal_id  = pos2id(End_Position(1), End_Position(2));

% % Creating a wall for a barrier within the path
% [bx, by] = meshgrid(randi([0, 50], 1, 20) / 10, randi([0, 50], 1, 20) / 10);
% 
% % Finding the node numbers of the barrier
% blocked = pos2id(bx(:), by(:));

% Randomly generating the boundaries
rng('Default') 
blocked = randi([1, total_nodes], Obstacle_number,1);

% Making sure the start and end aren't treated as obstacles
blocked(blocked == start_id | blocked == goal_id) = [];

% Finding the index of edges where nodes are blocked
bad = ismember(s, blocked) | ismember(t, blocked);

% Removing the edges of blocked nodes
s(bad) = [];  t(bad) = [];  weights(bad) = [];

% Regraphing the nodes and edges
graphObject = graph(s, t, weights);

% Searching through the nodes in the order they are discovered (First
% coulmn is from second is to: Events = (From, To))
events = dfsearch(graphObject, start_id, 'edgetonew');

% Parent pointers: parent(child) = node that discovered it
parent = zeros(total_nodes,1);
for k = 1:size(events,1)
    parent(events(k,2)) = events(k,1);
end

% Was the goal reached?
if goal_id ~= start_id && parent(goal_id) == 0
    error('Goal is not reachable from the start node.');
end

% Walk backward from goal to start
path = goal_id;
while path(1) ~= start_id
    path = [parent(path(1)); path];
end

% Plotting the map
plotObject = plot(graphObject,'XData',x,'YData',y);

% Styling the map
plotObject.Marker = 'o';
plotObject.MarkerSize = .5;
plotObject.NodeColor = 'b';
plotObject.LineWidth = .1;
plotObject.EdgeColor = 'k';
axis equal;
grid on;
ylabel('Y Position in Search Zone (m)')
xlabel('X Position in Search Zone (m)')
title('Depth First Algorithm')
hold off

% Doing Dijkstra's algorithm
[path_dijkstra, dijkstra_cost] = shortestpath(graphObject, start_id, goal_id);
eid_dijkstra = findedge(graphObject, path_dijkstra(1:end-1), path_dijkstra(2:end));

%% A* Search
h = @(node_id) hypot(x(goal_id) - x(node_id), y(goal_id) - y(node_id)); 
g_score   = inf(total_nodes, 1);
f_score   = inf(total_nodes, 1);
came_from = zeros(total_nodes, 1);

in_open   = false(total_nodes, 1);
in_closed = false(total_nodes, 1);

g_score(start_id) = 0;
f_score(start_id) = h(start_id);
in_open(start_id) = true; 

while any(in_open)

    temp_f = f_score;
    temp_f(~in_open) = inf;
    [~,current] = min(temp_f);

    if current == goal_id
        found = true;
        break
    end

    in_open(current) = false;
    in_closed(current) = true;

    nbrs = neighbors(graphObject, current);

    for k = 1:numel(nbrs)
        nb = nbrs(k);

        if in_closed(nb)
            continue
        end

        eid = findedge(graphObject, current, nb);
        edge_cost = graphObject.Edges.Weight(eid);

        temp_g = g_score(current) + edge_cost;
       
        if temp_g < g_score(nb)
            came_from(nb) = current;
            g_score(nb) = temp_g;
            f_score(nb) = g_score(nb) + h(nb);
            in_open(nb) = true;
        end
    end
end

path_A = goal_id;

while(path_A(1) ~= start_id)
    predecessor = came_from(path_A(1));
    if predecessor == 0
        error('Failed to reconstruct a valid path.');
    end
    path_A = [predecessor; path_A];
end


% Highlighting the paths
hold on
highlight(plotObject, path, 'NodeColor','r', 'EdgeColor','r', 'LineWidth',2); % Depth First
plot(nan, nan, 'Color','r','LineWidth',2)  % Depth First Legend Entry
% highlight(plotObject, path_dijkstra, 'NodeColor','g', 'EdgeColor','g', 'LineWidth',2); % Dijkstra
plot(x(path_dijkstra), y(path_dijkstra), 'g-',  'LineWidth', 3);
plot(nan, nan, 'Color','g','LineWidth',2) % Dijkstra Legend Entry
plot([Start_Position(1) End_Position(1)],[Start_Position(2) End_Position(2)], 'Color','m','LineStyle','--','LineWidth',2) % Straight Line
highlight(plotObject, blocked, 'NodeColor','k', 'MarkerSize',2); % Blocked Points
plot(nan, nan, 'MarkerFaceColor','k','MarkerSize',5,'LineStyle','none','Marker','o') % Blocked Points Legend
plot(x(start_id), y(start_id), 'gs', 'MarkerSize',10, 'MarkerFaceColor','g'); % Start Point
plot(x(goal_id),  y(goal_id),  'rp', 'MarkerSize',12, 'MarkerFaceColor','r'); % End Point
plot(nan, nan, 'Color','b','LineWidth',2) % Possible Path Legend Entry
% highlight(plotObject, path_A, 'NodeColor','cyan', 'EdgeColor','cyan', 'LineWidth',2,'LineStyle','--'); % A*
plot(x(path_A),        y(path_A),        'c--', 'LineWidth', 2);
hold off
print('DepthFirstTradeStufy','-dpng')

% plot(x(path_dijkstra), y(path_dijkstra), 'g-',  'LineWidth', 3);
plot(x(path_A),        y(path_A),        'c--', 'LineWidth', 2);
fprintf('Path cost:          %.6f m\n', g_score(goal_id));
fprintf('Dijkstra cost:      %.6f m\n', dijkstra_cost);

legend('','Depth First','Dijkstra','Straight Line','Blocked Points','Starting Point','End Point','Possible Path')
