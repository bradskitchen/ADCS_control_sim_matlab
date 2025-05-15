clear;
clc;
[radius, m_earth, grav_const, mu] = earth();


%% Initial Conditions

%------------------------------------------------------
altitude = 500000;
%------------------------------------------------------

%Meters
x_0 = radius + altitude;
y_0 = 0;
z_0 = 0;

%------------------------------------------------------
semi_major = norm([x_0; y_0; z_0]);

vcircular = sqrt(mu/semi_major);

inclination = 0;
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