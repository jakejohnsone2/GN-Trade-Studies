clear
clc
close all

% Initializing variables
Assumed_max_speed = 2;
Line_Dist = 5*sqrt(2);
Assumed_time = Line_Dist/Assumed_max_speed*2;
Maximum_dist = .3;
Mean_Accel = Assumed_max_speed/(.5*Assumed_time);

% Calculate the maximum allowable angle and acceleration
Max_angle = atand(Maximum_dist/Line_Dist);
Max_accel = 4*(Line_Dist+Maximum_dist)/Assumed_time^2;

% Calculating the errors of the acceleration and angle
Accel_Error = Max_accel-Mean_Accel;
Angle_Error = Max_angle;

% Calculating the STD
Accel_STD = Accel_Error/2;
Angle_STD = Angle_Error/2;

% Creating a normal gaussian of the errors
Angle_Dist = Angle_STD*randn(30,1);
Accel_Dist = Mean_Accel+Accel_STD*randn(30,1);

% Creating a meshgrid for the angle and acceleration errors
[ANGLE, ACCEL] = meshgrid(Angle_Dist,Accel_Dist);

% Calculating the total distance traveled with error
Distance_trav = ACCEL.*(Assumed_time^2/4);

% Finding the actual location of the error position
x_location = Distance_trav.*cosd(ANGLE);
y_location = Distance_trav.*sind(ANGLE);

% Calculating the distance between the endpoint and error point
Dest_Distance = sqrt((5-x_location).^2+(5-y_location).^2);

% Plotting the results
surf(ANGLE,ACCEL,Dest_Distance)


