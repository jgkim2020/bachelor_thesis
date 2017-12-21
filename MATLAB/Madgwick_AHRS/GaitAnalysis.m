%% Start of script
addpath('..\lib\Quaternions');      % include quaternion library
addpath('..\lib\Madgwick_AHRS');    % include Madgwick AHRS
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data
load('backward_wip.mat');
SamplePeriod = 1/100;
[time_step val] = size(Gyroscope);
time = 0:time_step - 1;
time = time.*SamplePeriod;

figure('Name', 'Sensor Data');
axis(1) = subplot(2,1,1);
hold on;
% Pelvis(1:3), Right Upper Leg(4:6), Right Lower Leg(7:9), Right Foot(10:12),
% Left Upper Leg(13:15), Left Lower Leg(16:18), Left Foot(19:21)
plot(time, Gyroscope(:,10), 'r');
plot(time, Gyroscope(:,11), 'g');
plot(time, Gyroscope(:,12), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (rad/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(2,1,2);
hold on;
% Pelvis(1:3), Right Upper Leg(4:6), Right Lower Leg(7:9), Right Foot(10:12),
% Left Upper Leg(13:15), Left Lower Leg(16:18), Left Foot(19:21)
plot(time, Accelerometer(:,10), 'r');
plot(time, Accelerometer(:,11), 'g');
plot(time, Accelerometer(:,12), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (m/s^{2})');
title('Accelerometer');
hold off;
% axis(3) = subplot(3,1,3);
% hold on;
% plot(time, Magnetometer(:,1), 'r');
% plot(time, Magnetometer(:,2), 'g');
% plot(time, Magnetometer(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Flux (G)');
% title('Magnetometer');
% hold off;
linkaxes(axis, 'x');

%% Process sensor data through algorithm

AHRS = MadgwickAHRS('SamplePeriod', SamplePeriod, 'Beta', 0.1);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.UpdateIMU(Gyroscope(t,10:12), Accelerometer(t,10:12));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ?0
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% End of script
