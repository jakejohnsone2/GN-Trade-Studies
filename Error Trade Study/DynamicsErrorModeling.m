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
vx  = 0.5;        % Nominal forward velocity (m/s)
vy  = 0;          % Tangential velocity (m/s)
theta = 0;        % Heading Angle (Rads)
r = 0;            % Yaw rate

K_omega = (Kx*R*B_w)/(2*Iz);

% State Space Matrices

A = [ 0,  0,  -vx*sin(theta)-vy*cos(theta),    cos(theta),   -sin(theta),    0;
    0,  0, vx*cos(theta)-vy*sin(theta),    sin(theta),   cos(theta),    0;
    0,  0,  0,    0,   0,    1;
    0,  0,  0, -Kx/m,  r,    vy;
    0,  0,  0,   -r,   0,  -vx;
    0,  0,  0,    0,   0, -Dr/Iz ];

B = [       0,           0;
    0,           0;
    0,           0;
    (Kx*R)/(2*m), (Kx*R)/(2*m);
    0,           0;
    -K_omega,      K_omega ];

% Converting to DT
ts = 20;

% Creating the ahat matrix
Ahat = [A B
    zeros(2,8)];

% Finding the DT representations of our state system
Ahat_exp =  expm(Ahat*ts);
F = Ahat_exp(1:6,1:6);
G = Ahat_exp(1:6,7:8);


%% Observability Matrix Calculation

% Gps only:
C_GPS = [1 0 0 0 0 0
    0 1 0 0 0 0];
D_GPS = [0 0
    0 0];

H_GPS = C_GPS;
M_GPS = D_GPS;



