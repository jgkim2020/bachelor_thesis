clear;
close all;
clc;
addpath('..\lib\Quaternions');
addpath('..\lib\ximu_matlab_library');
addpath('..\lib\AHRS');
addpath('..\lib\Mahony_AHRS');
addpath('..\lib\Madgwick_AHRS');
addpath('..\lib\Madgwick_SixDofAnimation');

% -------------------------------------------------------------------------
% Select dataset (comment in/out)

% filePath = 'Datasets/straightLine';
% startTime = 6;
% stopTime = 26;

% filePath = 'Datasets/stairsAndCorridor';
% startTime = 5;
% stopTime = 53;

% filePath = 'Datasets/spiralStairs';
% startTime = 4;
% stopTime = 47;

filePath = '..\Datasets\normal';
startTime = 0.2;
stopTime = 15.2;

% filePath = '..\Datasets\wip';
% startTime = 0.2;
% stopTime = 10.5;

% filePath = '..\Datasets\fast';
% startTime = 0.2;
% stopTime = 12.8;

% filePath = '..\Datasets\slow';
% startTime = 0.2;
% stopTime = 22.0;

% filePath = 'Datasets/forwardwip';
% startTime = 0.2;
% stopTime = 10.0;

% filePath = '..\Datasets\backwardwip';
% startTime = 0.2;
% stopTime = 9.4;

% load('..\Datasets\normal_walking.mat');
% startTime = 0.2;
% stopTime = 15.2;

% Pelvis(1:3), Right Upper Leg(4:6), Right Lower Leg(7:9), Right Foot(10:12),
% Left Upper Leg(13:15), Left Lower Leg(16:18), Left Foot(19:21)
% Accelerometer = Accelerometer/9.81; % acceleration in g
% Gyroscope = Gyroscope*180/pi; % angular velocity in dps
% Right Hip(1:3), Right Knee(4:6), Right Ankle(7:9), Left Hip(10:12), Left
% Knee(13:15), Left Ankle(16:18)
% JointAngle = JointAngle*1;

% Dataset for 6DOF tracking
% Accelerometer_sel = Accelerometer(:,10:12);
% Gyroscope_sel = Gyroscope(:,10:12);

% Dataset for joint angle tracking
% mode = 0; % 0 - right, 1 - left
% Accelerometer_sel1 = Accelerometer(:,1:3); % acceleration in g
% Gyroscope_sel1 = Gyroscope(:,1:3); % angular velocity in dps
% Accelerometer_sel2 = Accelerometer(:,(4:6) + 9*mode); % acceleration in g
% Gyroscope_sel2 = Gyroscope(:,(4:6) + 9*mode); % angular velocity in dps
% Accelerometer_sel3 = Accelerometer(:,(7:9) + 9*mode); % acceleration in g
% Gyroscope_sel3 = Gyroscope(:,(7:9) + 9*mode); % angular velocity in dps
% Accelerometer_sel4 = Accelerometer(:,(10:12) + 9*mode); % acceleration in g
% Gyroscope_sel4 = Gyroscope(:,(10:12) + 9*mode); % angular velocity in dps

% -------------------------------------------------------------------------
% Import data

samplePeriod = 1/100;
xIMUdata = xIMUdataClass(filePath, 'InertialMagneticSampleRate', 1/samplePeriod);
time = xIMUdata.CalInertialAndMagneticData.Time;
gyrX = xIMUdata.CalInertialAndMagneticData.Gyroscope.X;
gyrY = xIMUdata.CalInertialAndMagneticData.Gyroscope.Y;
gyrZ = xIMUdata.CalInertialAndMagneticData.Gyroscope.Z;
accX = xIMUdata.CalInertialAndMagneticData.Accelerometer.X;
accY = xIMUdata.CalInertialAndMagneticData.Accelerometer.Y;
accZ = xIMUdata.CalInertialAndMagneticData.Accelerometer.Z;
clear('xIMUdata');

% samplePeriod = 1/100;
% time = (0:samplePeriod:samplePeriod*(length(Accelerometer) - 1))';
% gyrX = Gyroscope_sel(:,1); gyrY = Gyroscope_sel(:,2); gyrZ = Gyroscope_sel(:,3);
% accX = Accelerometer_sel(:,1); accY = Accelerometer_sel(:,2); accZ = Accelerometer_sel(:,3);
% gyrX1 = Gyroscope_sel1(:,1); gyrY1 = Gyroscope_sel1(:,2); gyrZ1 = Gyroscope_sel1(:,3);
% accX1 = Accelerometer_sel1(:,1); accY1 = Accelerometer_sel1(:,2); accZ1 = Accelerometer_sel1(:,3);
% gyrX2 = Gyroscope_sel2(:,1); gyrY2 = Gyroscope_sel2(:,2); gyrZ2 = Gyroscope_sel2(:,3);
% accX2 = Accelerometer_sel2(:,1); accY2 = Accelerometer_sel2(:,2); accZ2 = Accelerometer_sel2(:,3);
% gyrX3 = Gyroscope_sel3(:,1); gyrY3 = Gyroscope_sel3(:,2); gyrZ3 = Gyroscope_sel3(:,3);
% accX3 = Accelerometer_sel3(:,1); accY3 = Accelerometer_sel3(:,2); accZ3 = Accelerometer_sel3(:,3);
% gyrX4 = Gyroscope_sel4(:,1); gyrY4 = Gyroscope_sel4(:,2); gyrZ4 = Gyroscope_sel4(:,3);
% accX4 = Accelerometer_sel4(:,1); accY4 = Accelerometer_sel4(:,2); accZ4 = Accelerometer_sel4(:,3);

% -------------------------------------------------------------------------
% Manually frame data

% startTime = 0;
% stopTime = 10;

indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time = time(indexSel);
gyrX = gyrX(indexSel, :); gyrY = gyrY(indexSel, :); gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :); accY = accY(indexSel, :); accZ = accZ(indexSel, :);
% gyrX1 = gyrX1(indexSel, :); gyrY1 = gyrY1(indexSel, :); gyrZ1 = gyrZ1(indexSel, :);
% accX1 = accX1(indexSel, :); accY1 = accY1(indexSel, :); accZ1 = accZ1(indexSel, :);
% gyrX2 = gyrX2(indexSel, :); gyrY2 = gyrY2(indexSel, :); gyrZ2 = gyrZ2(indexSel, :);
% accX2 = accX2(indexSel, :); accY2 = accY2(indexSel, :); accZ2 = accZ2(indexSel, :);
% gyrX3 = gyrX3(indexSel, :); gyrY3 = gyrY3(indexSel, :); gyrZ3 = gyrZ3(indexSel, :);
% accX3 = accX3(indexSel, :); accY3 = accY3(indexSel, :); accZ3 = accZ3(indexSel, :);
% gyrX4 = gyrX4(indexSel, :); gyrY4 = gyrY4(indexSel, :); gyrZ4 = gyrZ4(indexSel, :);
% accX4 = accX4(indexSel, :); accY4 = accY4(indexSel, :); accZ4 = accZ4(indexSel, :);

% -------------------------------------------------------------------------
% Detect stationary periods (Butterworth)

% Compute accelerometer magnitude
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

% HP filter accelerometer data
filtCutOff = 0.001;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);

% Compute absolute value
acc_magFilt = abs(acc_magFilt);

% LP filter accelerometer data
filtCutOff = 5;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);

% Threshold detection
stationary = acc_magFilt < 0.05;

% -------------------------------------------------------------------------
% Plot data raw sensor data and stationary periods

figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data');
ax(1) = subplot(2,1,1);
    hold on;
    plot(time, gyrX, 'r');
    plot(time, gyrY, 'g');
    plot(time, gyrZ, 'b');
    title('Gyroscope');
    xlabel('Time (s)');
    ylabel('Angular velocity (^\circ/s)');
    legend('X', 'Y', 'Z');
    hold off;
ax(2) = subplot(2,1,2);
    hold on;
    plot(time, accX, 'r');
    plot(time, accY, 'g');
    plot(time, accZ, 'b');
    plot(time, acc_magFilt, ':k');
    plot(time, stationary, 'k', 'LineWidth', 2);
    title('Accelerometer');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
    hold off;
linkaxes(ax,'x');

% -------------------------------------------------------------------------
% Compute orientation

quat = zeros(length(time), 4);
% quat1 = zeros(length(time), 4);
% quat2 = zeros(length(time), 4);
% quat3 = zeros(length(time), 4);
% quat4 = zeros(length(time), 4);

% Choose which AHRS algorithm to use
% AHRSalgorithm = AHRS('SamplePeriod', 1/100, 'Kp', 1, 'KpInit', 1);
% AHRSalgorithm1 = AHRS('SamplePeriod', 1/100, 'Kp', 1, 'KpInit', 1);
% AHRSalgorithm2 = AHRS('SamplePeriod', 1/100, 'Kp', 1, 'KpInit', 1);
% AHRSalgorithm3 = AHRS('SamplePeriod', 1/100, 'Kp', 1, 'KpInit', 1);
% AHRSalgorithm4 = AHRS('SamplePeriod', 1/100, 'Kp', 1, 'KpInit', 1);

% AHRSalgorithm = MahonyAHRS('SamplePeriod', 1/100, 'Kp', 1);

AHRSalgorithm = MadgwickAHRS('SamplePeriod', 1/100, 'Beta', 0.033);

% Initial convergence
initPeriod = 0.3;
indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
for i = 1:2000
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
    % AHRSalgorithm1.UpdateIMU([0 0 0], [mean(accX1(indexSel)) mean(accY1(indexSel)) mean(accZ1(indexSel))]);
    % AHRSalgorithm2.UpdateIMU([0 0 0], [mean(accX2(indexSel)) mean(accY2(indexSel)) mean(accZ2(indexSel))]);
    % AHRSalgorithm3.UpdateIMU([0 0 0], [mean(accX3(indexSel)) mean(accY3(indexSel)) mean(accZ3(indexSel))]);
    % AHRSalgorithm4.UpdateIMU([0 0 0], [mean(accX4(indexSel)) mean(accY4(indexSel)) mean(accZ4(indexSel))]);
end

% For all data
for t = 1:length(time)
    if(stationary(t))
        % AHRSalgorithm.Kp = 0.5;
        % AHRSalgorithm1.Kp = 0.5;
        % AHRSalgorithm2.Kp = 0.5;
        % AHRSalgorithm3.Kp = 0.5;
        % AHRSalgorithm4.Kp = 0.5;
    else
        % AHRSalgorithm.Kp = 0;
        % AHRSalgorithm1.Kp = 0;
        % AHRSalgorithm2.Kp = 0;
        % AHRSalgorithm3.Kp = 0;
        % AHRSalgorithm4.Kp = 0;
    end
    AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
    % AHRSalgorithm1.UpdateIMU(deg2rad([gyrX1(t) gyrY1(t) gyrZ1(t)]), [accX1(t) accY1(t) accZ1(t)]);
    % AHRSalgorithm2.UpdateIMU(deg2rad([gyrX2(t) gyrY2(t) gyrZ2(t)]), [accX2(t) accY2(t) accZ2(t)]);
    % AHRSalgorithm3.UpdateIMU(deg2rad([gyrX3(t) gyrY3(t) gyrZ3(t)]), [accX3(t) accY3(t) accZ3(t)]);
    % AHRSalgorithm4.UpdateIMU(deg2rad([gyrX4(t) gyrY4(t) gyrZ4(t)]), [accX4(t) accY4(t) accZ4(t)]);

    % output quaternion describing the sensor relative to the Earth
    % quat(t,:) = AHRSalgorithm.Quaternion; % for AHRS
    % quat1(t,:) = AHRSalgorithm1.Quaternion;
    % quat2(t,:) = AHRSalgorithm2.Quaternion;
    % quat3(t,:) = AHRSalgorithm3.Quaternion;
    % quat4(t,:) = AHRSalgorithm4.Quaternion;

    quat(t,:) = quaternConj(AHRSalgorithm.Quaternion); % for MadgwickAHRS, MahonyAHRS
end

% Calculate joint angle
% quaternion with respect to foot frame q_S1S2 = q_S1E x q_S2E*
% JointAngle = rad2deg(quatern2euler(quaternProd(quat2, quaternConj(quat1))));

% close all;
%% -------------------------------------------------------------------------
% Compute translational accelerations

% Rotate body accelerations to Earth frame
acc = quaternRotate([accX accY accZ], quaternConj(quat));

% % Remove gravity from measurements
% acc = acc - [zeros(length(time), 2) ones(length(time), 1)];     % unnecessary due to velocity integral drift compensation

% Convert acceleration measurements to m/s/s
acc = acc * 9.81;

% Plot translational accelerations
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Accelerations');
hold on;
plot(time, acc(:,1), 'r');
plot(time, acc(:,2), 'g');
plot(time, acc(:,3), 'b');
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s/s)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational velocities

acc(:,3) = acc(:,3) - 9.81;

% Integrate acceleration to yield velocity
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
    end
end


% Compute integral drift during non-stationary periods
velDrift = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == -1);
stationaryEnd = find([0; diff(stationary)] == 1);
for i = 1:numel(stationaryEnd)
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum = 1:(stationaryEnd(i) - stationaryStart(i));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end

% Remove integral drift
vel = vel - velDrift;

% Plot translational velocity
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
hold on;
plot(time, vel(:,1), 'r');
plot(time, vel(:,2), 'g');
plot(time, vel(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational position

% Integrate velocity to yield position
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;    % integrate velocity to yield position
end

% Plot translational position
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
hold on;
plot(time, pos(:,1), 'r');
plot(time, pos(:,2), 'g');
plot(time, pos(:,3), 'b');
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Plot 3D foot trajectory

% % Remove stationary periods from data to plot
% posPlot = pos(find(~stationary), :);
% quatPlot = quat(find(~stationary), :);
posPlot = pos;
% quatPlot = quat;
% quaternion with respect to foot frame q_S'S = q_S'E x q_SE*
quatPlot = quaternProd(quat, quaternConj(quat(1,:)));
% % quaternion with respect to foot frame (alternative method) R_SS' = transpose(R_ES)*R_ES'
% for i = 1:length(quatPlot)
%     quatPlot0(i,:) = rotMat2quatern(quatern2rotMat(quat(1,:))'*quatern2rotMat(quat(i,:)));
% end

% Extend final sample to delay end of animation
extraTime = 1;
onesVector = ones(extraTime*(1/samplePeriod), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];

% Create 6 DOF animation
SamplePlotFreq = 4;
Spin = 120;
SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', true, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
