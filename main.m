clear;
clc;
close all;
global BX BY BZ

%% setup igrf model
addpath m_IGRF\

%% get earth parameters
[radius, m_earth, grav_const, mu] = earth();


%% Initial Conditions

%------------------------------------------------------
altitude_0 = 500000;
%------------------------------------------------------

%Meters
x_0 = radius + altitude_0;
y_0 = 0;
z_0 = 0;

%------------------------------------------------------
semi_major = norm([x_0; y_0; z_0]);

vcircular = sqrt(mu/semi_major);

inclination = 56*pi/180;
%------------------------------------------------------

%Meters/sec
vel_x_0 = 0;
vel_y_0 = vcircular * cos(inclination); 
vel_z_0 = vcircular * sin(inclination);

%state vector
state_0 = [x_0; y_0; z_0; vel_x_0; vel_y_0; vel_z_0];


%% Time Window

period = 2*pi/sqrt(mu)*semi_major^(3/2);
number_of_orbits = 1;
tspan = [0 period*number_of_orbits];


%% Integrate equations of motion
[tout, stateout] = ode45(@cubesat, tspan, state_0);


%% loop through stateout to get mag field
BX_out = 0*stateout(:, 1);
BY_out = BX_out;
BZ_out = BX_out;

for idx = 1:length(tout)
    dstate_dt = cubesat(tout(idx), stateout(idx, :));
    BX_out(idx) = BX;
    BY_out(idx) = BY;
    BZ_out(idx) = BZ;
end


%% plot 3D orbit

%Extract position data
x = stateout(:, 1);
y = stateout(:, 2);
z = stateout(:, 3);

figure;

%plot orbit
plot3(x, y, z, 'r', linewidth=4);
hold on;


%load earth texture
load('topo.mat', 'topo', 'topomap1')

%plot the earth
[earth_x, earth_y, earth_z] = sphere(50);
surface(radius * earth_x, radius * earth_y, radius * earth_z, 'FaceColor', 'texturemap', 'CData', topo, 'EdgeColor','none');
colormap(topomap1)

%figure details
title("cubesat orbit");
xlabel('x');
ylabel('y');
zlabel('z');
axis equal;
grid on;
hold off

%%Plot Mag Field
figure;
hold on;
plot(tout, BX_out, 'b', LineWidth=2);
plot(tout, BY_out, 'g', LineWidth=2);
plot(tout, BZ_out, 'r', LineWidth=2);
grid on;
hold off