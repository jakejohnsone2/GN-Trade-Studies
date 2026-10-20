clear
clc
close all

%% Initial Calculations
% Initializing variables
Assumed_max_speed = 5;
Line_Dist = 5*sqrt(2);
Assumed_time = Line_Dist/Assumed_max_speed*2;
Maximum_dist = .3;
Mean_Accel = Assumed_max_speed/(.5*Assumed_time);
n = 300;

% Calculate the maximum allowable angle and acceleration
Max_angle = atand(Maximum_dist/Line_Dist);
Max_accel = 4*(Line_Dist+Maximum_dist)/Assumed_time^2;

% Calculating the errors of the acceleration and angle
Accel_Error = Max_accel-Mean_Accel;
Angle_Error = Max_angle;

% Calculating the STD
Accel_STD = Accel_Error/2;
Angle_STD = Angle_Error/2;

%% Creating Distributions and Calculating Error Distance
% Creating a normal gaussian of the errors
Angle_Dist = linspace(45-3*Angle_STD,45+3*Angle_STD,n);
Accel_Dist = linspace(Mean_Accel-3*Accel_STD,Mean_Accel+3*Accel_STD,n);

% Creating the optimal Plane
angle_plane = linspace(Angle_Dist(1),Angle_Dist(end),n);
accel_plane = linspace(Accel_Dist(1),Accel_Dist(end),n);
[ACCEL_p, ANGLE_p] = meshgrid(accel_plane,angle_plane);
Optimal_Dist_plane  = Maximum_dist*ones(size(ACCEL_p));

% Creating a meshgrid for the angle and acceleration errors
[ANGLE, ACCEL] = meshgrid(Angle_Dist,Accel_Dist);

% Calculating the total distance traveled with error
Distance_trav = ACCEL.*(Assumed_time^2/4);

% Finding the actual location of the error position
x_location = Distance_trav.*cosd(ANGLE);
y_location = Distance_trav.*sind(ANGLE);

% Calculating the distance between the endpoint and error point
Dest_Distance = sqrt((5-x_location).^2+(5-y_location).^2);


%% Plotting
% Plotting the results
figure()
hold on
surf(ANGLE-45,ACCEL-Mean_Accel,Dest_Distance,'EdgeColor','none')
colormap("turbo")
surf(ANGLE_p-45,ACCEL_p-Mean_Accel,Optimal_Dist_plane,'EdgeColor','none','FaceColor','m')
hold off
view(45,25)
colorbar
xlabel("Angle Error ({\circ})")
ylabel("Acceleration Error(m/s^2)")
zlabel("Error Distance (m)")
grid on
legend('Error Results','Max Error Distance')

%% Calculating the Circle


