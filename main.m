clear;
clc;
close all;
global BB inv_inertia inertia m lastMagUpdate nextMagUpdate lastSensorUpdate nextSensorUpdate
global BfieldMeasured pqrMeasured

tic

%% setup igrf model
addpath m_IGRF\

%% get earth parameters
[radius, m_earth, grav_const, mu] = earth();


%% Initial Conditions Position and Velocity

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

%% Initial Conditions for Attitude and Angular Velocity
phi_0 = 0;      %roll   (angle about x)
theta_0 = 0;    %pitch  (angle about y)
psi_0 = 0;      %yaw    (angle about z)

euler_angles_0 = [phi_0, theta_0, psi_0];
quanternions_0 = eul2quat(euler_angles_0, "XYZ");

p0 = 0;         %roll rate      (angular velocity about x)
q0 = 0;         %pitch rate     (angular velocity about y)
r0 = 0;         %yaw rate       (angular velocity about z)

%% state vector
state_0 = [x_0; y_0; z_0; vel_x_0; vel_y_0; vel_z_0; quanternions_0'; p0; q0; r0;];


%% Time Window

period = 2*pi/sqrt(mu)*semi_major^(3/2);
number_of_orbits = 1;
tspan = [0, period*number_of_orbits];
h = 1; % step size in seconds

%% Integrate equations of motion and collect data
disp('starting simulation');
disp(['orbital period: ', num2str(period), ' seconds']);
disp(['step size: ', num2str(h), 'seconds']);

%pre-calculate number of steps
num_steps = ceil((tspan(2) - tspan(1)) / h);
tout = zeros(num_steps + 1, 1);
stateout = zeros(num_steps + 1, length(state_0));
BX_out = zeros(num_steps + 1, 1);
BY_out = zeros(num_steps + 1, 1);
BZ_out = zeros(num_steps + 1, 1);
BX_Measured = zeros(num_steps + 1, 1);
BY_Measured = zeros(num_steps + 1, 1);
BZ_Measured = zeros(num_steps + 1, 1);
pqrm = zeros(num_steps + 1, 3);

tout(1) = tspan(1);
stateout(1, :) = state_0';

nextMagUpdate = 100;
lastMagUpdate = 0;

%call cubesat to initialize B fields
dstate_dt = cubesat(tout(1), state_0);
BX_out(1) = BB(1);
BY_out(1) = BB(2);
BZ_out(1) = BB(3);


%sensor parameters
lastSensorUpdate = 0;
nextSensorUpdate = 1;
[MagScaleBias, MagFieldBias, MagScaleNoise, MagFieldNoise, AngScaleBias, AngFieldBias, AngScaleNoise, AngFieldNoise] = sensor_params();

%status update
progress_interval = 1;
next_progress = progress_interval;
fprintf('sim progress: 0%%\n')

%Integration loop and mag field extration
for i = 1:num_steps
    t = tout(i);
    y = stateout(i, :)';

    %calculate RK4 coefficients
    k1 = cubesat(t, y);
    k2 = cubesat(t + h/2, y + h*(k1)/2);
    k3 = cubesat(t + h/2, y + h*(k2)/2);
    k4 = cubesat(t + h, y + h*(k3));

    %update time and state
    tout(i + 1) = t + h;
    stateout(i + 1, :) = y' + h/6 * (k1 + 2*(k2) + 2*(k3) + k4)';

    %get mag field at the new state
    dstate_dt = cubesat(tout(i + 1), stateout(i + 1, :)');
    BX_out(i + 1) = BB(1);
    BY_out(i + 1) = BB(2);
    BZ_out(i + 1) = BB(3);
    BX_Measured(i + 1) = BfieldMeasured(1);
    BY_Measured(i + 1) = BfieldMeasured(2);
    BZ_Measured(i + 1) = BfieldMeasured(3);

    %save polluted pqr signal
    pqrm(i, :) = pqrMeasured';

    %progress update
    progress = 100 * tout(i + 1) / tspan(2);
    if progress >= next_progress
        fprintf('Sim progress: %d%%\n', round(progress));
        next_progress = next_progress + progress_interval;
    end
end


%% plot 3D orbit

%Extract state vector
x = stateout(:, 1);
y = stateout(:, 2);
z = stateout(:, 3);
quanternions_out = stateout(:, 7:10);
euler_angles_out = quat2eul(quanternions_out);
pqr_out = stateout(:, 11:13);

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
plot(tout, BX_Measured, 'b--', LineWidth=2);
plot(tout, BY_Measured, 'g--', LineWidth=2);
plot(tout, BZ_Measured, 'r--', LineWidth=2);
grid on;
hold off

%%plot norm of B
Bnorm = sqrt(BX_out.^2 + BY_out.^2 + BZ_out.^2);
figure;
plot(tout, Bnorm, LineWidth=4);
grid on;

%%plot euler angles
figure;
plot(tout, euler_angles_out, LineWidth=2);

%%plot angular velocity
figure;
plot(tout, pqr_out, LineWidth=2);
hold on;
plot(tout, pqrm, '--', LineWidth=2);

toc