function z = sensor_suite(pose, pose_prev, k, dt, p)
%SENSOR_SUITE Stand-in sensors: wheel encoders, gyro, RTK GPS, magnetometer.
%   Owner: Sensors. Noise values are placeholders until the trade study
%   picks hardware. No cameras (excluded from autonomous travel).
%
%   pose, pose_prev : true [x; y; theta] now and one step ago
%   z.odom          : [v; omega] from encoders and gyro, every step
%   z.rtk           : [x; y] RTK fix, or [] between fixes
%   z.mag           : heading (rad), or [] between readings
%
%   Odometry comes from the true motion, not the plant's internals, so it
%   works unchanged with any D&C plant.

dp     = pose(1:2) - pose_prev(1:2);
th     = pose_prev(3);
v_true = ([cos(th), sin(th)] * dp) / dt;
w_true = wrap_pi(pose(3) - pose_prev(3)) / dt;

z.odom = [v_true + p.sens.sigma_v * randn;
          w_true + p.sens.sigma_w * randn];

z.rtk = [];
if mod(k, round(1 / (p.sens.rtk_rate * dt))) == 0
    z.rtk = pose(1:2) + p.sens.sigma_rtk * randn(2,1);
end

z.mag = [];
if mod(k, round(1 / (p.sens.mag_rate * dt))) == 0
    z.mag = wrap_pi(pose(3) + p.sens.sigma_mag * randn);
end
end
