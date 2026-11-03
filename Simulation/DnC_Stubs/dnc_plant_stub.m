function x = dnc_plant_stub(x, u, dt, p)
%DNC_PLANT_STUB Stand-in plant: unicycle with first-order speed response.
%   Owner: D&C. Replace with the real dynamics model.
%
%   x : [x; y; theta; v; omega]   u : [v_cmd; omega_cmd]
%   The speed lag is what makes the rover overshoot the stop, so e_stop is
%   not zero even with perfect state knowledge.

th = x(3);
v  = x(4);
w  = x(5);

v = v + (u(1) - v) * dt / p.rover.tau_v;
w = w + (u(2) - w) * dt / p.rover.tau_w;

v_ground = v * (1 + p.rover.slip_frac * randn);
w_ground = w + p.rover.yaw_slip * randn;

x = [x(1) + v_ground*cos(th)*dt;
     x(2) + v_ground*sin(th)*dt;
     wrap_pi(th + w_ground*dt);
     v;
     w];
end
