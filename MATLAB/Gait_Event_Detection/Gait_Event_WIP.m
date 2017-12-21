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

load('..\Datasets\normal_wip.mat');
% load('..\Datasets\forward_wip.mat');
% load('..\Datasets\backward_wip.mat');

Accelerometer = Accelerometer/9.81; % acceleration in g
Gyroscope = Gyroscope*180/pi; % angular velocity in dps

samplePeriod = 1/100;
time = (0:samplePeriod:samplePeriod*(length(Accelerometer) - 1))';

% Pelvis(1:3), Right Upper Leg(4:6), Right Lower Leg(7:9), Right Foot(10:12),
% Left Upper Leg(13:15), Left Lower Leg(16:18), Left Foot(19:21)
Accelerometer_sel = Accelerometer(:,10:12);
Gyroscope_sel = Gyroscope(:,10:12);
% Accelerometer_sel = Accelerometer(:,19:21);
% Gyroscope_sel = Gyroscope(:,19:21);

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
% Detect stationary periods (Madgwick method)

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
% Detect stationary periods (Madgwick method modified)
%
% % Compute accelerometer magnitude
% acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);
%
% % HP filter accelerometer data
% acc_magFilt_ = acc_mag - 1;
%
% % Compute absolute value
% acc_magFilt_ = abs(acc_magFilt_);
%
% % LP filter accelerometer data
% filtCutOff = 5;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% acc_magFilt_ = filter(b, a, acc_magFilt_);
%
% % Threshold detection
% stationary_ = acc_magFilt_ < 0.05;


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
% Detect WIP event (Foot-Off, Maximum-Height, Foot-Contact)

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

acc_ref_dot = zeros(length(gyr_ref),1);
gyr_ref_integ(1) = gyr_ref(1)*samplePeriod;
for i = 2:length(gyr_ref)
  gyr_ref_integ(i) = gyr_ref_integ(i - 1) + gyr_ref(i)*samplePeriod;
end

% Heuristic (functional analysis + FSM)
% FO_tick = 1; MH_tick = 1; FC_tick = 1;
% % state definition : FC -0-> FO -1-> MH -2->
% state = 0; % initial state
% flag_accup = 0;
% stationary1 = zeros(length(time),1);
% for i = 2:length(time)
%     switch state
%     case 0 % foot-contact ~ foot-off
%       % Reevaluate stationary periods
%       stationary1(i - 1) = 1;
%       % Detect foot-off
%       if(acc_ref(i - 1) > 1.1 && (acc_ref(i) - acc_ref(i - 1))/samplePeriod > 0)
%         FO_time(FO_tick, 1) = time(i - 1);
%         FO_tick = FO_tick + 1;
%         state = 1;
%       end
%
%     case 1 % foot-off ~ maximum-height
%       % Detect maximum-height
%       if(acc_ref(i - 1) > 1 && acc_ref(i) < 1)
%         MH_time(MH_tick, 1) = time(i - 1);
%         MH_tick = MH_tick + 1;
%         state = 2;
%       end
%
%     case 2 % maximum-height ~ foot-contact
%       % Detect foot-contact
%       if(acc_ref(i - 1) < 1 && acc_ref(i) > 1)
%         flag_accup = 1;
%       end
%       if(flag_accup == 1 && (acc_ref(i) - acc_ref(i - 1))/samplePeriod > 200)
%           FC_time(FC_tick, 1) = time(i - 1);
%           FC_tick = FC_tick + 1;
%           state = 0;
%       end
%
%     end
% end

% gyr_mag = sqrt(gyrX.*gyrX + gyrY.*gyrY + gyrZ.*gyrZ);
% acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);
% count = 0;
% for i = 2:length(time)
%     switch state
%     case 0 % foot-contact ~ foot-off
%       % Reevaluate stationary periods
%       stationary1(i - 1) = 1;
%       % Detect foot-off
%       if(gyr_mag(i - 1) > 30 && (gyr_mag(i) - gyr_mag(i - 1))/samplePeriod > 0)
%         count = count + 1;
%       else
%         count = 0;
%       end
%       if(count == 1)
%         FO_time(FO_tick, 1) = time(i - 1);
%         FO_tick = FO_tick + 1;
%         state = 1;
%         count = 0;
%       end
%
%     case 1 % foot-off ~ foot-contact
%       % Detect foot-contact
%       if(gyr_mag(i - 1) < 15)
%         count = count + 1;
%         if(count == 3)
%           FC_time(FC_tick, 1) = time(i - 1);
%           FC_tick = FC_tick + 1;
%           state = 0;
%           count = 0;
%         end
%       else
%         count = 0;
%       end
%
%     end
% end


FO_tick = 1; FC_tick = 1;
% state definition : FC -0-> FO -1->
state = 0;
% LP Filter
filtCutOff = 5;
[b_LP, a_LP] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');

stationary_(1) = 1;
acc_magFilt_HP(1) = 0;
acc_magFilt_LP(1) = 0;
acc_magFilt_(1) = 0;

for i = 2:length(time)
  % HP Filter
  acc_magFilt_HP(i) = acc_mag(i) - 1;

  acc_magFilt_HP(i) = abs(acc_magFilt_HP(i));

  % LP Filter
  acc_magFilt_LP(i) = b_LP(1)*acc_magFilt_HP(i) + b_LP(2)*acc_magFilt_HP(i - 1) - a_LP(2)*acc_magFilt_LP(i - 1);
  acc_magFilt_(i) = acc_magFilt_LP(i);

  if(acc_magFilt_(i) < 0.05)
    stationary_(i) = 1;
    if(state == 1)
      state = 0;
      FC_time(FC_tick, 1) = time(i - 1);
      FC_tick = FC_tick + 1;
    end
  else
    stationary_(i) = 0;
    if(state == 0)
      state = 1;
      FO_time(FO_tick, 1) = time(i - 1);
      FO_tick = FO_tick + 1;
    end
  end
end


%% -------------------------------------------------------------------------
% Plot raw sensor data (principal axis only) with gait event detection
% figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data: Gyroscope, Accelerometer');
% ax(1) = subplot(2,1,1);
%     hold on;
%     % plot(time, gyr_ref);
%     plot(time, gyr_mag)
%     plot(time, 100*stationary_, 'k', 'LineWidth', 2);
%     title('Gyroscope (principal axis)');
%     xlabel('Time (s)');
%     ylabel('Angular velocity (^\circ/s)');
%     % vline(FC_time, 'k', 'FC');
%     % vline(FO_time, 'r', 'FO');
%     % vline(MH_time, 'g', 'MH');
%     vline(FC_time, 'k');
%     vline(FO_time, ':r');
%     % vline(MH_time, ':g');
%     hold off;
% ax(2) = subplot(2,1,2);
%     hold on;
%     % plot(time, acc_ref);
%     plot(time, acc_mag);
%     plot(time, stationary, 'k', 'LineWidth', 2);
%     title('Accelerometer (principal axis)');
%     xlabel('Time (s)');
%     ylabel('Acceleration (g)');
%     % vline(FC_time, 'k', 'FC');
%     % vline(FO_time, 'r', 'FO');
%     % vline(MH_time, 'g', 'MH');
%     vline(FC_time, 'k');
%     vline(FO_time, ':r');
%     % vline(MH_time, ':g');
%     hold off;
% linkaxes(ax,'x');

%% -------------------------------------------------------------------------
% Plot acceleration magnitude & filtfilt/myfilter
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Angular Velocity and Acceleration');
ax(1) = subplot(2,1,1);
    hold on;
    plot(time, acc_magFilt);
    plot(time, stationary, 'k', 'LineWidth', 2);
    title('MATLAB filtfilt() function (forward-backward zero-phase digital filtering)');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    hold off;
ax(2) = subplot(2,1,2);
    hold on;
    plot(time, acc_magFilt_);
    plot(time, stationary_, 'k', 'LineWidth', 2);
    title('custom filter (real-time forward filtering)');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
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
%     title('Gyroscope');
%     xlabel('Time (s)');
%     ylabel('Angular velocity (^\circ/s)');
%     legend('X', 'Y', 'Z');
%     hold off;
% ax(2) = subplot(2,1,2);
%     hold on;
%     plot(time, accX, 'r');
%     plot(time, accY, 'g');
%     plot(time, accZ, 'b');
%     title('Accelerometer');
%     xlabel('Time (s)');
%     ylabel('Acceleration (g)');
%     legend('X', 'Y', 'Z');
%     hold off;
% linkaxes(ax,'x');

% %% -------------------------------------------------------------------------
% Plot acceleration and jerk
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
