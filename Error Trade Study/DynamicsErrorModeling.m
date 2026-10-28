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
    fprintf('System is Observable with pure GPS\n');
else
    fprintf('System is not Observable with pure GPS\n');
    UNOB_GPS = null(O_GPS);
end


% Magnetometer:
C_Mag = [0 0 1 0 0 0];
D_Mag = [0 0];

H_GPS = C_Mag;
M_GPS = D_Mag;

% Check Observability of Magnetometer Alone
O_Mag = obsv(F,H_GPS);
if rank(O_Mag) == size(F,1)
    fprintf('System is Observable with pure Magnetometer\n');
else
    fprintf('System is not Observable with pure Magnetometer\n');
    UNOB_GPS = null(O_GPS);
end

% IMU:
C_IMU = [0 0 0 1 0 0
    0 0 0 0 1 0];
D_IMU = [0 0
    0 0];

H_GPS = C_IMU;
M_GPS = D_IMU;

% Check Observability of IMU Alone
O_IMU = obsv(F,H_GPS);
if rank(O_IMU) == size(F,1)
    fprintf('System is Observable with pure IMU\n');
else
    fprintf('System is not Observable with pure IMU\n');
    UNOB_GPS = null(O_IMU);
end

% Gyro:
C_Gyro = [0 0 0 1 0 0
    0 0 0 0 1 0];
D_Gyro = [0 0
    0 0];

H_GPS = C_Gyro;
M_GPS = D_Gyro;

% Check Observability of Gyro Alone
O_Gyro = obsv(F,H_GPS);
if rank(O_Gyro) == size(F,1)
    fprintf('System is Observable with pure Gyro\n');
else
    fprintf('System is not Observable with pure Gyro\n');
    UNOB_GPS = null(O_Gyro);
end


% Magnetometer and GPS:
C_MG = [1 0 0 0 0 0
    0 1 0 0 0 0
    0 0 1 0 0 0];
D_MG = [0 0
    0 0];

H_MG = C_MG;
M_MG = D_MG;

% Check Observability of Gyro Alone
O_MG = obsv(F,H_MG);
if rank(O_MG) == size(F,1)
    fprintf('System is Observable with Magnetometer and GPS\n');
else
    fprintf('System is not Observable with Magnetometer and GPS\n');
    UNOB_GPS = null(O_MG);
end


% Magnetometer and IMU:
C_MI = [0 0 0 1 0 0
    0 0 0 0 1 0
    0 0 1 0 0 0];
D_MI = [0 0
    0 0];

H_MI = C_MI;
M_MI = D_MI;

% Check Observability of Gyro Alone
O_MI = obsv(F,H_MI);
if rank(O_MI) == size(F,1)
    fprintf('System is Observable with Magnetometer and IMU\n');
else
    fprintf('System is not Observable with Magnetometer and IMU\n');
    UNOB_GPS = null(O_MI);
end

% Gyro and GPS:
C_GG = [1 0 0 0 0 0
    0 1 0 0 0 0
    0 0 0 0 0 1];
D_GG = [0 0
    0 0];

H_MG = C_GG;
M_MG = D_GG;

% Check Observability of Gyro Alone
O_GG = obsv(F,H_MG);
if rank(O_GG) == size(F,1)
    fprintf('System is Observable with Gyro and GPS\n');
else
    fprintf('System is not Observable with Gyro and GPS\n');
    UNOB_GPS = null(O_GG);
end


% Gyro and IMU:
C_GI = [0 0 0 1 0 0
    0 0 0 0 1 0
    0 0 0 0 0 1];
D_GI = [0 0
    0 0];

H_GI = C_GI;
M_GI = D_GI;

% Check Observability of Gyro Alone
O_GI = obsv(F,H_GI);
if rank(O_GI) == size(F,1)
    fprintf('System is Observable with Gyro and IMU\n');
else
    fprintf('System is not Observable with Gyro and IMU\n');
    UNOB_GPS = null(O_GI);
end

% Gyro, Magnetometer, and IMU:
C_GIM = [0 0 0 1 0 0
    0 0 0 0 1 0
    0 0 0 0 0 1
    0 0 1 0 0 0];
D_GIM = [0 0
    0 0];

H_GIM = C_GIM;
M_GIM = D_GIM;

% Check Observability of Gyro Alone
O_GIM = obsv(F,H_GIM);
if rank(O_GIM) == size(F,1)
    fprintf('System is Observable with Gyro, IMU, and Magnetometer\n');
else
    fprintf('System is not Observable with Gyro, IMU, and Magnetometer\n');
    UNOB_GPS = null(O_GIM);
end

% Gyro, Magnetometer, and GPS:
C_GGM = [1 0 0 0 0 0
    0 1 0 0 0 0
    0 0 0 0 0 1
    0 0 1 0 0 0];
D_GGM = [0 0
    0 0];

H_GGM = C_GGM;
M_GGM = D_GGM;

% Check Observability of Gyro Alone
O_GGM = obsv(F,H_GGM);
if rank(O_GGM) == size(F,1)
    fprintf('System is Observable with Gyro, GPS, and Magnetometer\n');
else
    fprintf('System is not Observable with Gyro, GPS, and Magnetometer\n');
    UNOB_GPS = null(O_GGM);
end

% Check Observability of Gyro Alone
O_GGM = obsv(F,H_GIM);
if rank(O_GGM) == size(F,1)
    fprintf('System is Observable with Gyro, IMU, and Magnetometer\n');
else
    fprintf('System is not Observable with Gyro, IMU, and Magnetometer\n');
    UNOB_GPS = null(O_GGM);
end

% Everything:
C_all = [1 0 0 0 0 0
    0 1 0 0 0 0
    0 0 0 0 0 1
    0 0 1 0 0 0
    0 0 0 1 0 0
    0 0 0 0 1 0];
D_all = [0 0
    0 0];

H_all = C_all;
M_all = D_all;

% Check Observability of Gyro Alone
O_all = obsv(F,H_all);
if rank(O_all) == size(F,1)
    fprintf('System is Observable with Gyro, GPS, Magnetometer, and IMU\n');
else
    fprintf('System is not Observable with Gyro, GPS, Magnetometer, and IMU\n');
    UNOB_GPS = null(O_all);
end

% Pulling the ranks of the obervability matrices
Observable_States = [rank(O_GPS) rank(O_Mag) rank(O_IMU) rank(O_Gyro) ...
    rank(O_MG) rank(O_MI) rank(O_GG) rank(O_GI) rank(O_GIM) rank(O_GGM) ...
    rank(O_GIM) rank(O_all)];

% Creating Labels
sensorLabels = {'GPS','Magnetometer','IMU','Gyro','Magnetometer + GPS', ...
    'Magnetometer + IMU','Gyro + GPS','Gyro + IMU','Gyro + Magnetometer + IMU', ...
    'Gyro + GPS + Magnetometer','Gyro + Magnetometer + IMU','All Sensors'};

% Creating Bar Graph
figure()
bar(Observable_States)
yline(6,'Color','r','LineWidth',2,'LineStyle','--','Label','Fully Observable System','LabelHorizontalAlignment','left')
set(gca, 'XTickLabel', sensorLabels);
ylim([0 8])
ylabel('Observable States')
title('Sensors vs Observable States')
print('Observable_States','-dpng')