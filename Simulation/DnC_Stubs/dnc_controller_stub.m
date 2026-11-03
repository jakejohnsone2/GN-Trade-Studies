function [u, cstate] = dnc_controller_stub(x_fb, path, cstate, p)
%DNC_CONTROLLER_STUB Stand-in stop-and-pivot controller.
%   Owner: D&C. Replace with the real controller.
%
%   x_fb   : pose the controller is allowed to see, [x; y; theta]. This is
%            x_hat or the true pose, set by p.sim.feedback.
%   path   : planner output (path.wp, Nx2 waypoints)
%   cstate : controller memory; [] on the first call
%   u      : [v_cmd; omega_cmd]
%
%   At each waypoint: pivot in place to the next leg's heading, then drive
%   the leg straight. The final leg ramps speed down to p.guid.v_f and
%   commands a stop at the goal (3.2.1.1 standard final approach).

if isempty(cstate)
    cstate = struct('idx', 2, 'mode', 'pivot', 'done', false);
end
if cstate.done
    u = [0; 0];
    return
end

wp    = path.wp;
A     = wp(cstate.idx-1, :)';
B     = wp(cstate.idx, :)';
L     = norm(B - A);
d     = (B - A) / L;
psi   = atan2(d(2), d(1));
final = cstate.idx == size(wp, 1);
th    = x_fb(3);

if strcmp(cstate.mode, 'pivot')
    err = wrap_pi(psi - th);
    if abs(err) > p.guid.pivot_tol
        u = [0; sat(2.0 * err, p.rover.omega_max)];
        return
    end
    cstate.mode = 'drive';
end

% Drive the leg
r         = x_fb(1:2) - A;
remaining = L - d' * r;
cross     = d(1)*r(2) - d(2)*r(1);

if remaining <= 0
    u = [0; 0];
    if final
        cstate.done = true;
    else
        cstate.idx  = cstate.idx + 1;
        cstate.mode = 'pivot';
    end
    return
end

if final
    v = p.guid.v_f + (p.rover.v_max - p.guid.v_f) * min(1, remaining / p.guid.L_f);
else
    v = max(p.guid.v_min, min(p.rover.v_max, remaining));
end
psi_cmd = psi - atan(p.guid.k_cross * cross);
w       = sat(p.guid.k_heading * wrap_pi(psi_cmd - th), p.rover.omega_max);
u       = [v; w];
end

%% ------------------------------------------------------------------------
function y = sat(y, lim)
y = max(-lim, min(lim, y));
end
