%PATH_TEST Exercise plan_path with several start/goal conditions and plot results.
% Run from any folder; uses the simulation's standard parameters and map.

% Locate this script and add the project folders so params and plan_path can
% be found even when MATLAB's current folder is somewhere else.
this_dir = fileparts(mfilename('fullpath'));
sim_dir = fileparts(this_dir);
addpath(genpath(sim_dir));

% Load the same map, clearance, and final-leg requirement used by missions.
p = params();

% Each row defines one named test scenario and its [x y] start and goal.
% The last case crosses the obstacle field to exercise route-around behavior.
cases = {
    'Default route',       p.map.start(1:2), p.marker.true_pos;
    'Near map edge',       [0.10 4.80],     [4.80 0.10];
    'Across obstacle field',[0.50 2.50],    [4.50 2.50]
};

% Give every scenario its own axes so the paths can be compared side by side.
figure('Name','Planner path checks','Color','w');
for k = 1:size(cases,1)
    % Pull the name and coordinates from this row of the cases table.
    start_xy = cases{k,2};
    goal_xy = cases{k,3};

    % Configure a map view with the same bounds and units as the simulation.
    subplot(1,size(cases,1),k);
    hold on; axis equal; grid on; box on;
    xlim([0 p.map.size]); ylim([0 p.map.size]);
    title(cases{k,1}); xlabel('East (m)'); ylabel('North (m)');

    % Draw each obstacle and its expanded clearance boundary. The path should
    % stay outside the dotted clearance rings, not merely outside the circles.
    ang = linspace(0,2*pi,100);
    for j = 1:size(p.map.obstacles,1)
        c = p.map.obstacles(j,1:2);
        r = p.map.obstacles(j,3);
        fill(c(1)+r*cos(ang), c(2)+r*sin(ang), [0.65 0.65 0.65], ...
            'EdgeColor','none');
        plot(c(1)+(r+p.map.clearance)*cos(ang), ...
             c(2)+(r+p.map.clearance)*sin(ang), ':', 'Color',[0.4 0.4 0.4]);
    end

    % Mark endpoints before planning so they remain visible if planning fails.
    plot(start_xy(1),start_xy(2),'go','MarkerFaceColor','g');
    plot(goal_xy(1),goal_xy(2),'rp','MarkerFaceColor','r');
    % Run the planner and draw its waypoint polyline. Catch a no-path or input
    % error so the remaining scenarios still run and display their outcomes.
    try
        path = plan_path(p.map,start_xy,goal_xy,p);
        plot(path.wp(:,1),path.wp(:,2),'b-o','LineWidth',1.2);
        legend('Obstacle','Clearance','Start','Goal','Path','Location','best');
        % Print useful diagnostics for comparing routes and final-leg compliance.
        fprintf('%s: %d waypoints, A* cost %.2f m, final leg OK: %d\n', ...
            cases{k,1},size(path.wp,1),path.cost,path.final_ok);
    catch ME
        % Put the failure message in the corresponding subplot and command
        % window instead of aborting the whole test script.
        text(0.05,0.95,ME.message,'Units','normalized','Color','r', ...
            'VerticalAlignment','top','Interpreter','none');
        fprintf('%s: no path (%s)\n',cases{k,1},ME.message);
    end
end
