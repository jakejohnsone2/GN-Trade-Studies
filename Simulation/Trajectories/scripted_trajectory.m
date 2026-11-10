function traj = scripted_trajectory(name, p)
%   SCRIPTED_TRAJECTORY Known true motion for localization studies (3.2.2).
%   traj = scripted_trajectory(name, p)
%
%   Replaces the controller + plant with a scripted path, so the sensors and
%   estimator can be studied before D&C delivers a model. The rover only
%   drives straight, arcs or pivots in place (no sideways slip).
%
%   name
%     'stationary'  sit still (gyro drift, RTK/mag noise floor, lock time)
%     'straight'    drive straight across the map
%     'square'      straight legs with pivot turns between them
%     'arc'         constant-radius circle (steady turn)
%     'figure8'     two opposite circles (heading excites both directions)
%     'mission'     the planned path to the marker, at cruise speed, with
%                   pivots at waypoints (the geometry the real mission sees)
%
%   traj.t     1xN time (s), traj.t(1) = 0
%   traj.pose  3xN true [x; y; theta], ENU, theta from East
%   traj.v     1xN true forward speed (m/s)
%   traj.w     1xN true yaw rate (rad/s)
%   traj.name  the name given
%
%   Each trajectory is a list of segments [duration v w]. Add your own case
%   below; everything downstream only reads traj.

v  = p.traj.v;          % (m/s) cruise speed
wp = p.traj.w_pivot;    % (rad/s) pivot rate
start = p.map.start(:);

switch lower(name)
    case 'stationary'
        seg = [p.traj.t_hold, 0, 0];

    case 'straight'
        L   = p.map.size - 2*start(1);
        seg = [L/v, v, 0];

    case 'square'
        L   = p.traj.side;
        leg = [L/v, v, 0];
        piv = [(pi/2)/wp, 0, wp];
        seg = [leg; piv; leg; piv; leg; piv; leg];

    case 'arc'
        R   = p.traj.radius;
        seg = [2*pi*R/v, v, v/R];

    case 'figure8'
        R   = p.traj.radius;
        seg = [2*pi*R/v, v, v/R;
               2*pi*R/v, v, -v/R];

    case 'mission'
        path = plan_path(p.map, start(1:2), p.marker.true_pos(:), p);
        seg  = segments_from_waypoints(path.wp, start(3), v, wp);

    otherwise
        error('scripted_trajectory: unknown trajectory "%s".', name);
end

% Settle at the end so the filter can be checked at rest
seg = [seg; p.traj.t_settle, 0, 0];

traj      = integrate_segments(seg, start, p.sim.dt);
traj.name = name;
end

%% ------------------------------------------------------------------------
function seg = segments_from_waypoints(wpts, th0, v, wp)
% Pivot to face each next waypoint, then drive straight to it.
seg = zeros(0, 3);
th  = th0;
for i = 1:size(wpts,1) - 1
    d      = wpts(i+1,:) - wpts(i,:);
    L      = norm(d);
    if L < 1e-9, continue; end
    th_new = atan2(d(2), d(1));
    dth    = wrap_pi(th_new - th);
    if abs(dth) > 1e-6
        seg = [seg; abs(dth)/wp, 0, sign(dth)*wp]; %#ok<AGROW>
    end
    seg = [seg; L/v, v, 0]; %#ok<AGROW>
    th  = th_new;
end
end

%% ------------------------------------------------------------------------
function traj = integrate_segments(seg, start, dt)
% Exact arc integration, so the truth has no discretization error.
n_per = max(1, round(seg(:,1) / dt));
N     = sum(n_per) + 1;
pose  = zeros(3, N);
vv    = zeros(1, N);
ww    = zeros(1, N);
pose(:,1) = start;
k = 1;
for s = 1:size(seg,1)
    v = seg(s,2);
    w = seg(s,3);
    for j = 1:n_per(s)
        th = pose(3,k);
        if abs(w) < 1e-9
            dxy = v*dt*[cos(th); sin(th)];
        else
            dxy = (v/w)*[sin(th + w*dt) - sin(th);
                         cos(th) - cos(th + w*dt)];
        end
        pose(:,k+1) = [pose(1:2,k) + dxy; wrap_pi(th + w*dt)];
        vv(k+1) = v;
        ww(k+1) = w;
        k = k + 1;
    end
end
traj.t    = (0:N-1) * dt;
traj.pose = pose;
traj.v    = vv;
traj.w    = ww;
end
