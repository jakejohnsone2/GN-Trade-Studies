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
theta = pi/3;        % Heading Angle (Rads)
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

% Compare Eigenvalues for Open-Loop Response Sample Limit
[~,OL_eig] = eig(A);

% Observe Eigenvalues, any Complex Eigenvalues Dictate Sample Frequency
if any(imag(OL_eig))
    disp('Open-Loop Sample Time Limited by Complex Modes');
end

% Converting to DT
ts = 0.01;

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

% Check Observability of GPS Alone
O_GPS = obsv(F,H_GPS);
if rank(O_GPS) == size(F,1)
    fprintf('System is Observable with pure GPS');
else
    fprintf('System is not Observable with pure GPS');
    UNOB_GPS = null(O_GPS);
end
