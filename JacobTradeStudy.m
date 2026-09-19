clear
close all
clc

% =========================================================================
% FULL MATLAB SCRIPT: GENERATING A COMPLETE SPATIAL GRID OF NODES & EDGES
% =========================================================================

% 1. Define your grid parameters
grid_spacing = 5;  % The exact distance between each node (in meters)
num_x = 100;          % Number of nodes along the horizontal axis (columns)
num_y = 100;          % Number of nodes along the vertical axis (rows)

% 2. Generate the spatial coordinates (Every grid point becomes a node)
[X, Y] = meshgrid(0:grid_spacing:(num_x-1)*grid_spacing, ...
                  0:grid_spacing:(num_y-1)*grid_spacing);

% Flatten matrices into single column vectors for the graph
x_meters = X(:);
y_meters = Y(:);
num_nodes = length(x_meters);

% 3. Programmatically connect adjacent grid nodes (Horizontal & Vertical Edges)
s = [];
t = [];

for r = 1:num_y
    for c = 1:num_x
        % Calculate the unique 1D index of the current node
        current_node = (c-1)*num_y + r;
        
        % Connect to the neighbor on the RIGHT (Horizontal Edge)
        if c < num_x
            neighbor_right = c*num_y + r;
            s = [s; current_node];
            t = [t; neighbor_right];
        end
        
        % Connect to the neighbor ABOVE (Vertical Edge)
        if r < num_y
            neighbor_up = current_node + 1;
            s = [s; current_node];
            t = [t; neighbor_up];
        end
    end
end

% 4. Assign edge weights (All straight grid steps equal the grid_spacing)
weights = ones(size(s)) * grid_spacing;

% 5. Build the MATLAB Graph Object
G = graph(s, t, weights);

% 6. Plot the Complete Gridded Map Environment
figure('Color', 'w');
hold on;

% Plot using the generated grid coordinates
p = plot(G, 'XData', x_meters, 'YData', y_meters);

% Style the grid map
p.Marker = 'o';                     % Circular nodes at every point
p.MarkerSize = 2;
p.NodeColor = [0 0.5 0.8];          % Vibrant blue nodes
p.LineWidth = 1.5;
p.EdgeColor = [0.7 0.7 0.7];        % Light gray grid lines

% Label every node with its index number for easy reference
% p.NodeLabel = cellstr(num2str((1:num_nodes)'));

% Configure strict spatial accuracy
grid on;
axis equal; % CRITICAL: Forces 1 meter on X to look identical to 1 meter on Y
xlim([-grid_spacing, num_x * grid_spacing]);
ylim([-grid_spacing, num_y * grid_spacing]);

title(sprintf('Full %dx%d Grid Map (%d Nodes, Spacing: %dm)', ...
    num_x, num_y, num_nodes, grid_spacing), 'FontSize', 12);
xlabel('X Position (meters)', 'FontSize', 10);
ylabel('Y Position (meters)', 'FontSize', 10);

hold off;


% %% dfsearch_pathfinding_example.m
% % Demonstrates using MATLAB's dfsearch function to find a path
% % between a source node and a target node in a graph.
% 
% %% 1. Build a sample graph
% % Nodes 1-8, edges defined below
% s = [1 1 1 2 2 3];
% t = [2 3 4 3 4 4];
% G = graph(s, t);
% 
% figure;
% p = plot(G, 'Layout', 'layered');
% title('Sample Graph');
% 
% %% 2. Define source and target nodes
% sourceNode = 1;
% targetNode = 8;
% 
% %% 3. Run dfsearch, flagging 'edgetonew' events
% % Each row of treeEdges = [fromNode toNode], in the order DFS
% % discovers them. Because dfsearch always adds the discovering
% % edge as it visits a new node, these edges form a DFS tree that
% % we can walk backwards from the target to the source.
% treeEdges = dfsearch(G, sourceNode, 'edgetonew');
% 
% %% 4. Build a predecessor map from the DFS tree edges
% predecessor = zeros(1, numnodes(G));
% for i = 1:size(treeEdges, 1)
%     fromNode = treeEdges(i, 1);
%     toNode   = treeEdges(i, 2);
%     predecessor(toNode) = fromNode;
% end
% 
% %% 5. Reconstruct the path from target back to source
% path = targetNode;
% current = targetNode;
% while current ~= sourceNode
%     current = predecessor(current);
%     if current == 0
%         error('No path found from node %d to node %d.', sourceNode, targetNode);
%     end
%     path = [current, path]; %#ok<AGROW>
% end
% 
% fprintf('Path found: %s\n', mat2str(path));
% 
% %% 6. Highlight the path on the plot
% highlight(p, path, 'EdgeColor', 'r', 'LineWidth', 2, 'NodeColor', 'r');