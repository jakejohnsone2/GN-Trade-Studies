%% ARES ROVER GUIDANCE AND NAVIGATION TESTING
% Authors: Guidance and Navigation
clc
clear 
close all

%% 1. SYSTEM PARAMETERS
m   = 10.0;       % Rover mass (kg)
Iz  = 2.0;        % Yaw moment of inertia (kg*m^2)
R   = 0.15;       % Effective wheel radius (m)
B_w = 0.40;       % Half-track width (m)
Kx  = 200.0;      % Longitudinal traction stiffness
Dr  = 10.0;       % Yaw damping coefficient
v0  = 0.5;        % Nominal forward velocity (m/s)

K_omega = (Kx*R*B_w)/(2*Iz);

% State Space Matrices

A = [ 0,  0,  0,    1,   0,    0;
    0,  0, v0,    0,   1,    0;
    0,  0,  0,    0,   0,    1;
    0,  0,  0, -Kx/m,  0,    0;
    0,  0,  0,    0,   0,  -v0;
    0,  0,  0,    0,   0, -Dr/Iz ];

B = [       0,           0;
    0,           0;
    0,           0;
    (Kx*R)/(2*m), (Kx*R)/(2*m);
    0,           0;
    -K_omega,      K_omega ];

