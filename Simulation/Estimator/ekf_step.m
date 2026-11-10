function [x_hat, P] = ekf_step(x_hat, P, z, dt, p)
%   EKF Function
%   Owner: Xander
%
%   TODO (3.2.2.1): model RTK latency and check update rate against the

%% Predict from odometry
v  = z.odom(1);
w  = z.odom(2);
th = x_hat(3);

x_hat = x_hat + [v*cos(th); v*sin(th); w] * dt;
x_hat(3) = wrap_pi(x_hat(3));

F = [1 0 -v*sin(th)*dt;
     0 1  v*cos(th)*dt;
     0 0  1];
B = [cos(th)*dt 0;
     sin(th)*dt 0;
     0          dt];
Q = B * diag([p.sens.sigma_v^2, p.sens.sigma_w^2]) * B';
P = F*P*F' + Q;

%% Correct with RTK position
if ~isempty(z.rtk)
    H = [1 0 0; 0 1 0];
    R = p.sens.sigma_rtk^2 * eye(2);
    [x_hat, P] = correct(x_hat, P, z.rtk - H*x_hat, H, R);
end

%% Correct with magnetometer heading
if ~isempty(z.mag)
    H = [0 0 1];
    R = p.sens.sigma_mag^2;
    [x_hat, P] = correct(x_hat, P, wrap_pi(z.mag - x_hat(3)), H, R);
end
end

%% ------------------------------------------------------------------------
function [x, P] = correct(x, P, innov, H, R)
S    = H*P*H' + R;
K    = P*H' / S;
x    = x + K*innov;
x(3) = wrap_pi(x(3));
I_KH = eye(3) - K*H;
P    = I_KH*P*I_KH' + K*R*K';               % Joseph form keeps P symmetric
end
