clear
close all
clc

Total_size = 5; % (m) Size of search are
n = 30; % Total intervals for mapping (Change for more complexity)
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

% Plotting the map
plotObject = plot(graphObject,'XData',x,'YData',y);

% Styling the map
plotObject.Marker = 'o';
plotObject.MarkerSize = .5;
plotObject.NodeColor = 'b';
plotObject.LineWidth = .5;
plotObject.EdgeColor = 'k';
axis equal;
grid on;
ylabel('Y Position in Search Zone (m)')
xlabel('X Position in Search Zone (m)')
title('Gridded Map of Nodes')
hold off

%% Creating the Boundaries
% Creating a function that calulates the node number
pos2id = @(px,py) round(py/Spacing)*Nodes + round(px/Spacing) + 1;

% Calculating the node number of starting and end position
start_id = pos2id(Start_Position(1), Start_Position(2));
goal_id  = pos2id(End_Position(1), End_Position(2));

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

% Highlighting the blocked points
highlight(plotObject, blocked, 'NodeColor','k', 'MarkerSize',2);