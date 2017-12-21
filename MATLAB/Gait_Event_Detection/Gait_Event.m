clear;
close all;
clc;
addpath('..\lib\hline_vline');
addpath('..\lib\Quaternions');
addpath('..\lib\AHRS');
addpath('..\lib\Mahony_AHRS');
addpath('..\lib\Madgwick_AHRS');

% -------------------------------------------------------------------------
% Import data

% load('..\Datasets\normal_walking.mat');
% load('..\Datasets\fast_walking.mat');
load('..\Datasets\slow_walking.mat');

Accelerometer = Accelerometer/9.81; % acceleration in g
Gyroscope = Gyroscope*180/pi; % angular velocity in dps

samplePeriod = 1/100;
time = (0:samplePeriod:samplePeriod*(length(Accelerometer) - 1))';

% Pelvis(1:3), Right Upper Leg(4:6), Right Lower Leg(7:9), Right Foot(10:12),
% Left Upper Leg(13:15), Left Lower Leg(16:18), Left Foot(19:21)
% Accelerometer_sel = Accelerometer(:,10:12);
% Gyroscope_sel = Gyroscope(:,10:12);
Accelerometer_sel = Accelerometer(:,19:21);
Gyroscope_sel = Gyroscope(:,19:21);

gyrX = Gyroscope_sel(:,1); gyrY = Gyroscope_sel(:,2); gyrZ = Gyroscope_sel(:,3);
accX = Accelerometer_sel(:,1); accY = Accelerometer_sel(:,2); accZ = Accelerometer_sel(:,3);


% -------------------------------------------------------------------------
% Manually frame data
startTime = time(1);
stopTime = time(length(time));
indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time = time(indexSel);
gyrX = gyrX(indexSel, :); gyrY = gyrY(indexSel, :); gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :); accY = accY(indexSel, :); accZ = accZ(indexSel, :);


% -------------------------------------------------------------------------
% Reorient frame of reference (sensor->Earth)
% quat = zeros(length(time), 4);
%
% AHRSalgorithm = AHRS('SamplePeriod', 1/100, 'Kp', 1, 'KpInit', 1);
% % AHRSalgorithm = MahonyAHRS('SamplePeriod', 1/100, 'Kp', 1);
% % AHRSalgorithm = MadgwickAHRS('SamplePeriod', 1/100, 'Beta', 0.033);
%
% initPeriod = 0.3;
% indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
% for i = 1:2000
%     AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
% end
%
% for t = 1:length(time)
%     if(stationary(t))
%         AHRSalgorithm.Kp = 0.5;
%     else
%         AHRSalgorithm.Kp = 0;
%     end
%     AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
%
%     % output quaternion describing the sensor relative to the Earth (q_SE)
%     quat(t,:) = AHRSalgorithm.Quaternion; % for AHRS
%     % quat(t,:) = quaternConj(AHRSalgorithm.Quaternion); % for MadgwickAHRS, MahonyAHRS
% end
%
% gyr = quaternRotate([gyrX gyrY gyrZ], quaternConj(quat)); % q_SE*w_S*q_ES
% acc = quaternRotate([accX accY accZ], quaternConj(quat)); % q_SE*a_S*q_ES
%
% gyrX = gyr(:,1); gyrY = gyr(:,2); gyrZ = gyr(:,3);
% accX = acc(:,1); accY = acc(:,2); accZ = acc(:,3);


% -------------------------------------------------------------------------
% Reorient frame of reference (sensor->reference sensor)
quat = zeros(length(time), 4);

% AHRSalgorithm = AHRS('SamplePeriod', 1/100, 'Kp', 1, 'KpInit', 1);
% AHRSalgorithm = MahonyAHRS('SamplePeriod', 1/100, 'Kp', 1);
AHRSalgorithm = MadgwickAHRS('SamplePeriod', 1/100, 'Beta', 0.033);

initPeriod = 0.3;
indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
for i = 1:2000
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
end

for t = 1:length(time)
    % if(stationary(t))
    %     AHRSalgorithm.Kp = 0.5;
    % else
    %     AHRSalgorithm.Kp = 0;
    % end
    AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);

    % output quaternion describing the sensor relative to the Earth (q_SE)
    % quat(t,:) = AHRSalgorithm.Quaternion; % for AHRS
    quat(t,:) = quaternConj(AHRSalgorithm.Quaternion); % for MadgwickAHRS, MahonyAHRS
end

gyr = quaternRotate([gyrX gyrY gyrZ], quaternConj(quat(1,:))); % q_SE*w_S*q_ES0
acc = quaternRotate([accX accY accZ], quaternConj(quat(1,:))); % q_SE*a_S*q_ES0

gyrX = gyr(:,1); gyrY = gyr(:,2); gyrZ = gyr(:,3);
accX = acc(:,1); accY = acc(:,2); accZ = acc(:,3);


% -------------------------------------------------------------------------
% Time derivative of sensor data
val = length(gyrX);
gyrX_dot = zeros(val,1); gyrY_dot = zeros(val,1); gyrZ_dot = zeros(val,1);
accX_dot = zeros(val,1); accY_dot = zeros(val,1); accZ_dot = zeros(val,1);
for i = 2:val
    gyrX_dot(i) = (gyrX(i) - gyrX(i - 1))/samplePeriod;
    gyrY_dot(i) = (gyrY(i) - gyrY(i - 1))/samplePeriod;
    gyrZ_dot(i) = (gyrZ(i) - gyrZ(i - 1))/samplePeriod;
    accX_dot(i) = (accX(i) - accX(i - 1))/samplePeriod;
    accY_dot(i) = (accY(i) - accY(i - 1))/samplePeriod;
    accZ_dot(i) = (accZ(i) - accZ(i - 1))/samplePeriod;
end


% -------------------------------------------------------------------------
% Detect stationary periods (Madgwick method)

% Compute accelerometer magnitude
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);
gyr_mag = sqrt(gyrX.*gyrX + gyrY.*gyrY + gyrZ.*gyrZ);

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
% Detect gait event (Heel-Strike, Foot-Flat, Heel-Off, Toe-Off)

% Determine principal axis
gyr_amplitude = [max(gyrX) - min(gyrX), max(gyrY) - min(gyrY), max(gyrZ) - min(gyrZ)];
[val, gyr_principal] = max(gyr_amplitude);
acc_amplitude = [max(accX) - min(accX), max(accY) - min(accY), max(accZ) - min(accZ)];
[val, acc_principal] = max(acc_amplitude);

if(gyr_principal == 1)
    gyr_ref = gyrX;
elseif(gyr_principal == 2)
    gyr_ref = gyrY;
elseif(gyr_principal == 3)
    gyr_ref = gyrZ;
end

if(acc_principal == 1)
    acc_ref = accX;
elseif(acc_principal == 2)
    acc_ref = accY;
elseif(acc_principal == 3)
    acc_ref = accZ;
end

% gyr_ref_integ = zeros(length(gyr_ref),1);
% gyr_ref_integ(1) = gyr_ref(1)*samplePeriod;
% for i = 2:length(gyr_ref)
%   gyr_ref_integ(i) = gyr_ref_integ(i - 1) + gyr_ref(i)*samplePeriod;
% end

% Heuristic (functional analysis + FSM)
HS_tick = 1; FF_tick = 1; HO_tick = 1; TO_tick = 1;
% state definition : FF -0-> HO -1-> TO -2-> HS -3-> FF
state = 0; % initial state
flag_gyrup = 0; flag_gyrdown = 0;
err0_cnt = 0;
stationary1 = zeros(length(time),1);

for i = 2:length(time)
    switch state
    case 0 % foot-flat ~ heel-off
      % Reevaluate stationary periods
      stationary1(i - 1) = 1;
      % Detect heel-off
      if(gyr_ref(i - 1) > 30 && (gyr_ref(i) - gyr_ref(i - 1))/samplePeriod > 0)
          HO_time(HO_tick, 1) = time(i - 1);
          HO_tick = HO_tick + 1;
          state = 1;
      end

    case 1 % heel-off ~ toe-off
      % Detect toe-off
      if(gyr_ref(i - 1) > 0 && gyr_ref(i) < 0 )
        flag_gyrdown = 1;
      end
      if(flag_gyrdown == 1 && (gyr_ref(i) - gyr_ref(i - 1))/samplePeriod < -2000)
          TO_time(TO_tick, 1) = time(i - 1);
          TO_tick = TO_tick + 1;
          state = 2;
          flag_gyrdown = 0;
      end

    case 2 % toe-off ~ heel-strike
      % Detect heel-strike
      if(gyr_ref(i - 1) < 0 && gyr_ref(i) > 0)
        flag_gyrup = 1;
      end
      if(flag_gyrup == 1 && (gyr_ref(i) - gyr_ref(i - 1))/samplePeriod > 2000)
        HS_time(HS_tick, 1) = time(i - 1);
        HS_tick = HS_tick + 1;
        state = 3;
        flag_gyrup = 0;
      end
      % Detect Gait Abort
      if(gyr_ref(i - 1) > 0)
        err0_cnt = err0_cnt + 1;
        if(err0_cnt > 0)
          FF_time(FF_tick, 1) = time(i - 1);
          FF_tick = FF_tick + 1;
          state = 0;
          flag_gyrup = 0;
          err0_cnt = 0;
        end
      end

    case 3 % heel-strike ~ foot-flat
      % Detect foot-flat
      if(abs(gyr_ref(i - 1)) < 10 && (gyr_ref(i) - gyr_ref(i - 1))/samplePeriod < 0)
        FF_time(FF_tick, 1) = time(i - 1);
        FF_tick = FF_tick + 1;
        state = 0;
      end

    end
end


%% -------------------------------------------------------------------------
% Plot raw sensor data (principal axis only) with gait event detection
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data: Gyroscope, Accelerometer');
ax(1) = subplot(2,1,1);
    hold on;
    plot(time, gyr_ref);
    % plot(time, gyr_mag)
    plot(time, 100*stationary, 'k', 'LineWidth', 2);
    title('Gyroscope (principal axis)');
    xlabel('Time (s)');
    ylabel('Angular velocity (^\circ/s)');
%     vline(HS_time, 'k', 'HS');
%     vline(FF_time, 'r', 'FF');
%     vline(HO_time, 'g', 'HO');
%     vline(TO_time, 'b', 'TO');
    vline(HS_time, ':k');
    vline(FF_time, ':r');
    vline(HO_time, ':g');
    vline(TO_time, ':b');
    hold off;
ax(2) = subplot(2,1,2);
    hold on;
    % plot(time, acc_ref);
    plot(time, acc_mag);
    plot(time, stationary1, 'k', 'LineWidth', 2);
    title('Accelerometer (principal axis)');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
%     vline(HS_time, 'k', 'HS');
%     vline(FF_time, 'r', 'FF');
%     vline(HO_time, 'g', 'HO');
%     vline(TO_time, 'b', 'TO');
    vline(HS_time, ':k');
    vline(FF_time, ':r');
    vline(HO_time, ':g');
    vline(TO_time, ':b');
    hold off;
linkaxes(ax,'x');

%% -------------------------------------------------------------------------
% Plot raw sensor data (gyro, acc)
% figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Angular Velocity and Acceleration');
% ax(1) = subplot(2,1,1);
%     hold on;
%     plot(time, gyrX, 'r');
%     plot(time, gyrY, 'g');
%     plot(time, gyrZ, 'b');
%     xlabel('Time (s)');
%     ylabel('Angular velocity (^\circ/s)');
%     legend('X', 'Y', 'Z');
%     hold off;
% ax(2) = subplot(2,1,2);
%     hold on;
%     plot(time, accX, 'r');
%     plot(time, accY, 'g');
%     plot(time, accZ, 'b');
%     xlabel('Time (s)');
%     ylabel('Acceleration (g)');
%     legend('X', 'Y', 'Z');
%     hold off;
% linkaxes(ax,'x');

% %% -------------------------------------------------------------------------
% Plot angular velocity and angular acceleration (gyro)
% figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Angular Velocity and Angular Acceleration');
% ax(1) = subplot(2,1,1);
%     hold on;
%     plot(time, gyrX, 'r');
%     plot(time, gyrY, 'g');
%     plot(time, gyrZ, 'b');
%     xlabel('Time (s)');
%     ylabel('Angular velocity (^\circ/s)');
%     legend('X', 'Y', 'Z');
%     hold off;
% ax(2) = subplot(2,1,2);
%     hold on;
%     plot(time, gyrX_dot, 'r');
%     plot(time, gyrY_dot, 'g');
%     plot(time, gyrZ_dot, 'b');
%     xlabel('Time (s)');
%     ylabel('Angular acceleration (^\circ/s^{2})');
%     legend('X', 'Y', 'Z');
%     hold off;
% linkaxes(ax,'x');

% %% -------------------------------------------------------------------------
% Plot angular velocity and angle (gyro, principal axis only)
% figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Angular Velocity and Angular Acceleration');
% ax(1) = subplot(2,1,1);
%     hold on;
%     plot(time, gyr_ref);
%     title('Gyroscope (principal axis)');
%     xlabel('Time (s)');
%     ylabel('Angular velocity (^\circ/s)');
%     hold off;
% ax(2) = subplot(2,1,2);
%     hold on;
%     plot(time, gyr_ref_integ);
%     title('Gyroscope (principal axis)');
%     xlabel('Time (s)');
%     ylabel('Angle (^\circ)');
%     hold off;
% linkaxes(ax,'x');

% %% -------------------------------------------------------------------------
% % Plot acceleration and jerk
% figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Acceleration and Jerk');
% ax(1) = subplot(2,1,1);
%     hold on;
%     plot(time, accX, 'r');
%     plot(time, accY, 'g');
%     plot(time, accZ, 'b');
%     xlabel('Time (s)');
%     ylabel('Acceleration (g)');
%     legend('X', 'Y', 'Z');
%     hold off;
% ax(2) = subplot(2,1,2);
%     hold on;
%     plot(time, accX_dot, 'r');
%     plot(time, accY_dot, 'g');
%     plot(time, accZ_dot, 'b');
%     xlabel('Time (s)');
%     ylabel('Jerk (g/s)');
%     legend('X', 'Y', 'Z');
%     hold off;
% linkaxes(ax,'x');
