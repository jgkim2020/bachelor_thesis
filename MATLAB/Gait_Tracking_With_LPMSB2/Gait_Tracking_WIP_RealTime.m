clear;
close all;
clc;
addpath('..\lib\hline_vline');
addpath('..\lib\Quaternions');
addpath('..\lib\AHRS');
addpath('..\lib\Mahony_AHRS');
addpath('..\lib\Madgwick_AHRS');
addpath('..\lib\SixDofAnimation');


% -------------------------------------------------------------------------
% Import data (from LPMS-B2)
% -------------------------------------------------------------------------

% filepath = '..\Datasets\lpms_0728\';
% filename = 'wip_neutral';
% csvdata = csvread([filepath,filename,'.csv'], 1, 0);

filepath = '..\Datasets\lpms_1006\';
filename = 'dorsal_100Hz_1';
csvdata = csvread([filepath,filename,'.csv'], 1, 0);

% column headers
% 01 SensorId, TimeStamp, FrameNumber, AccX (g), AccY (g), AccZ (g), GyroX (deg/s),
% 08 GyroY (deg/s), GyroZ (deg/s), MagX (uT), MagY (uT), MagZ (uT), EulerX (deg),
% 14 EulerY (deg), EulerZ (deg), QuatW, QuatX, QuatY, QuatZ, LinAccX (g), LinAccY (g),
% 22 LinAccZ (g), Pressure (kPa), Altitude (m), Temperature (degC), HeaveMotion (m)
number_of_sensor = 3;
samplePeriod = 1/100;
data_length = length(csvdata);
SensorId = csvdata(:, 1);
FrameNumber = csvdata(:, 3);

% Check data integrity
Sensor1_Idx = find(SensorId == 1);
Sensor2_Idx = find(SensorId == 2);
Sensor3_Idx = find(SensorId == 3);
if(FrameNumber(max(Sensor1_Idx)) + 1 ~= length(Sensor1_Idx))
  warning('sensor1 packet loss!');
end
if(FrameNumber(max(Sensor2_Idx)) + 1 ~= length(Sensor2_Idx))
  warning('sensor2 packet loss!');
end
if(FrameNumber(max(Sensor3_Idx)) + 1 ~= length(Sensor3_Idx))
  warning('sensor3 packet loss!');
end
maxFrame = min([length(Sensor1_Idx) length(Sensor2_Idx) length(Sensor3_Idx)]);
if(length(Sensor3_Idx) == 0)
    maxFrame = min([length(Sensor1_Idx) length(Sensor2_Idx)]);
end

lpms_TimeStamp = zeros(maxFrame, number_of_sensor);
lpms_AccX = zeros(maxFrame, number_of_sensor);
lpms_AccY = zeros(maxFrame, number_of_sensor);
lpms_AccZ = zeros(maxFrame, number_of_sensor);
lpms_GyroX = zeros(maxFrame, number_of_sensor);
lpms_GyroY = zeros(maxFrame, number_of_sensor);
lpms_GyroZ = zeros(maxFrame, number_of_sensor);
lpms_QuatW = zeros(maxFrame, number_of_sensor);
lpms_QuatX = zeros(maxFrame, number_of_sensor);
lpms_QuatY = zeros(maxFrame, number_of_sensor);
lpms_QuatZ = zeros(maxFrame, number_of_sensor);
lpms_LinAccX = zeros(maxFrame, number_of_sensor);
lpms_LinAccY = zeros(maxFrame, number_of_sensor);
lpms_LinAccZ = zeros(maxFrame, number_of_sensor);
for i = 1:data_length
  idx = FrameNumber(i) + 1;
  if(idx <= maxFrame)
    lpms_TimeStamp(idx, SensorId(i)) = csvdata(i, 2);
    lpms_AccX(idx, SensorId(i)) = -csvdata(i, 4); lpms_AccY(idx, SensorId(i)) = -csvdata(i, 5); lpms_AccZ(idx, SensorId(i)) = -csvdata(i, 6);
    lpms_GyroX(idx, SensorId(i)) = csvdata(i, 7); lpms_GyroY(idx, SensorId(i)) = csvdata(i, 8); lpms_GyroZ(idx, SensorId(i)) = csvdata(i, 9);
    lpms_QuatW(idx, SensorId(i)) = csvdata(i, 16); lpms_QuatX(idx, SensorId(i)) = csvdata(i, 17);
    lpms_QuatY(idx, SensorId(i)) = csvdata(i, 18); lpms_QuatZ(idx, SensorId(i)) = csvdata(i, 19);
    lpms_LinAccX(idx, SensorId(i)) = csvdata(i, 20); lpms_LinAccY(idx, SensorId(i)) = csvdata(i, 21); lpms_LinAccZ(idx, SensorId(i)) = csvdata(i, 22);
  end
end

% Manually frame data
startTime = lpms_TimeStamp(1, 1);
stopTime = lpms_TimeStamp(maxFrame, 1);
indexSel = find(sign(lpms_TimeStamp(:,1)-startTime)+1, 1) : find(sign(lpms_TimeStamp(:,1)-stopTime)+1, 1);
maxFrame = length(indexSel);
lpms_TimeStamp = lpms_TimeStamp(indexSel, :);
lpms_AccX = lpms_AccX(indexSel, :); lpms_AccY = lpms_AccY(indexSel, :); lpms_AccZ = lpms_AccZ(indexSel, :);
lpms_GyroX = lpms_GyroX(indexSel, :); lpms_GyroY = lpms_GyroY(indexSel, :); lpms_GyroZ = lpms_GyroZ(indexSel, :);
lpms_QuatW = lpms_QuatW(indexSel, :); lpms_QuatX = lpms_QuatX(indexSel, :);
lpms_QuatY = lpms_QuatY(indexSel, :); lpms_QuatZ = lpms_QuatZ(indexSel, :);
lpms_LinAccX = lpms_LinAccX(indexSel, :); lpms_LinAccY = lpms_LinAccY(indexSel, :); lpms_LinAccZ = lpms_LinAccZ(indexSel, :);

% Repackage data
lpms_Acc = zeros(maxFrame, 3, number_of_sensor);
lpms_Gyro = zeros(maxFrame, 3, number_of_sensor);
lpms_Quat = zeros(maxFrame, 4, number_of_sensor);
lpms_LinAcc = zeros(maxFrame, 3, number_of_sensor);
for j = 1:number_of_sensor
  lpms_Acc(:,:,j) = [lpms_AccX(:,j) lpms_AccY(:,j) lpms_AccZ(:,j)];
  lpms_Gyro(:,:,j) = [lpms_GyroX(:,j) lpms_GyroY(:,j) lpms_GyroZ(:,j)];
  lpms_Quat(:,:,j) = [lpms_QuatW(:,j) lpms_QuatX(:,j) lpms_QuatY(:,j) lpms_QuatZ(:,j)];
  lpms_LinAcc(:,:,j) = [lpms_LinAccX(:,j) lpms_LinAccY(:,j) lpms_LinAccZ(:,j)];
end


% -------------------------------------------------------------------------
% Compare filter (accelerometer + gyroscope)
% -------------------------------------------------------------------------

% % Detect stationary periods (Madgwick's method, forward-backward filtering)
% % Compute accelerometer magnitude
% acc_mag = sqrt(lpms_AccX.*lpms_AccX + lpms_AccY.*lpms_AccY + lpms_AccZ.*lpms_AccZ);
% % HP filter accelerometer data
% filtCutOff = 0.001;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
% acc_magFilt_Mad1 = filtfilt(b, a, acc_mag);
% % Compute absolute value
% acc_magFilt_Mad1 = abs(acc_magFilt_Mad1);
% % LP filter accelerometer data
% filtCutOff = 5;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% acc_magFilt_Mad1 = filtfilt(b, a, acc_magFilt_Mad1);
% % Threshold detection
% stationary_Mad1 = acc_magFilt_Mad1 < 0.05;
% % Compute gyroscope magnitude
% gyro_mag = sqrt(lpms_GyroX.*lpms_GyroX + lpms_GyroY.*lpms_GyroY + lpms_GyroZ.*lpms_GyroZ);
% % HP filter gyroscope data
% filtCutOff = 0.001;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
% gyro_magFilt_Mad1 = filtfilt(b, a, gyro_mag);
% % Compute absolute value
% gyro_magFilt_Mad1 = abs(gyro_magFilt_Mad1);
% % LP filter gyroscope data
% filtCutOff = 5;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% gyro_magFilt_Mad1 = filtfilt(b, a, gyro_magFilt_Mad1);
% % Threshold detection
% stationary_Mad1 = stationary_Mad1.*(gyro_magFilt_Mad1 < 100);
%
% % Detect stationary periods (Madgwick's method, forward filtering)
% % Compute accelerometer magnitude
% acc_mag = sqrt(lpms_AccX.*lpms_AccX + lpms_AccY.*lpms_AccY + lpms_AccZ.*lpms_AccZ);
% % HP filter accelerometer data
% filtCutOff = 0.01;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
% acc_magFilt_Mad2 = filter(b, a, acc_mag);
% acc_magFilt_Mad2 = acc_mag - 1;
% % Compute absolute value
% acc_magFilt_Mad2 = abs(acc_magFilt_Mad2);
% % LP filter accelerometer data
% filtCutOff = 5;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% acc_magFilt_Mad2 = filter(b, a, acc_magFilt_Mad2);
% % Threshold detection
% stationary_Mad2 = acc_magFilt_Mad2 < 0.05;
% % Compute gyroscope magnitude
% gyro_mag = sqrt(lpms_GyroX.*lpms_GyroX + lpms_GyroY.*lpms_GyroY + lpms_GyroZ.*lpms_GyroZ);
% % HP filter gyroscope data
% filtCutOff = 0.01;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
% gyro_magFilt_Mad2 = filter(b, a, gyro_mag);
% gyro_magFilt_Mad2 = gyro_mag;
% % LP filter gyroscope data
% filtCutOff = 5;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% gyro_magFilt_Mad2 = filter(b, a, gyro_magFilt_Mad2);
% % Threshold detection
% stationary_Mad2 = stationary_Mad2.*(gyro_magFilt_Mad2 < 100);
%
% % Detect stationary periods (custom method, forward filtering)
% % Compute accelerometer magnitude
% acc_mag = sqrt(lpms_AccX.*lpms_AccX + lpms_AccY.*lpms_AccY + lpms_AccZ.*lpms_AccZ);
% % Compute filter coeffiecients
% filtCutOff = 5;
% [b_LP, a_LP] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% % Initialize variable
% acc_magFilt_HP = zeros(maxFrame, number_of_sensor);
% acc_magFilt_LP = zeros(maxFrame, number_of_sensor);
% for i = 2:maxFrame
%   % HP filter accelerometer data
%   acc_magFilt_HP(i,:) = acc_mag(i,:) - 1;
%   % Compute absolute value
%   acc_magFilt_HP(i,:) = abs(acc_magFilt_HP(i,:));
%   % LP filter accelerometer data
%   acc_magFilt_LP(i,:) = b_LP(1)*acc_magFilt_HP(i, :) + b_LP(2)*acc_magFilt_HP(i-1, :) - a_LP(2)*acc_magFilt_LP(i-1, :);
% end
% % Rename variable
% acc_magFilt = acc_magFilt_LP;
% % Threshold detection
% stationary = acc_magFilt < 0.05;
% % Compute gyroscope magnitude
% gyro_mag = sqrt(lpms_GyroX.*lpms_GyroX + lpms_GyroY.*lpms_GyroY + lpms_GyroZ.*lpms_GyroZ);
% % Compute filter coeffiecients
% filtCutOff = 5;
% [b_LP, a_LP] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% % Initialize variable
% gyro_magFilt_HP = zeros(maxFrame, number_of_sensor);
% gyro_magFilt_LP = zeros(maxFrame, number_of_sensor);
% for i = 2:maxFrame
%   % HP filter gyroscope data
%   gyro_magFilt_HP(i,:) = gyro_mag(i,:);
%   % LP filter gyroscope data
%   gyro_magFilt_LP(i,:) = b_LP(1)*gyro_magFilt_HP(i, :) + b_LP(2)*gyro_magFilt_HP(i-1, :) - a_LP(2)*gyro_magFilt_LP(i-1, :);
% end
% % Rename variable
% gyro_magFilt = gyro_magFilt_LP;
% % Threshold detection
% stationary = stationary.*(gyro_magFilt < 100);
%
% % -------------------------------------------------------------------------
% % Plot magnitude of acceleration/angular velocity & filtfilt, filter, custom
% for plotSensor = 1:number_of_sensor
%   figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', ['Compare filter performance : SensorId = ',num2str(plotSensor)]);
%   ax(1) = subplot(2,1,1);
%       hold on;
%       plot(lpms_TimeStamp(:,plotSensor), acc_mag(:,plotSensor));
%       plot(lpms_TimeStamp(:,plotSensor), gyro_mag(:,plotSensor).*0.01);
%       plot(lpms_TimeStamp(:,plotSensor), stationary_Mad1(:,plotSensor), 'k', 'LineWidth', 2);
%       title('MATLAB filtfilt() function (forward-backward zero-phase digital filtering)');
%       xlabel('Time (s)');
%       ylabel('Acceleration (g), Angular velocity (100dps)');
%       hold off;
%   ax(2) = subplot(2,1,2);
%       hold on;
%       plot(lpms_TimeStamp(:,plotSensor), acc_mag(:,plotSensor));
%       plot(lpms_TimeStamp(:,plotSensor), gyro_mag(:,plotSensor).*0.01);
%       plot(lpms_TimeStamp(:,plotSensor), stationary(:,plotSensor), 'k', 'LineWidth', 2);
%       title('custom filter (real-time forward filtering)');
%       xlabel('Time (s)');
%       ylabel('Acceleration (g), Angular velocity (100dps)');
%       hold off;
%   linkaxes(ax,'x');
% end


% -------------------------------------------------------------------------
% main
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% Initialize variable
acc_mag = sqrt(lpms_AccX.*lpms_AccX + lpms_AccY.*lpms_AccY + lpms_AccZ.*lpms_AccZ);
acc_magFilt_HP = zeros(maxFrame, number_of_sensor);
acc_magFilt_LP = zeros(maxFrame, number_of_sensor);
acc_magFilt = zeros(maxFrame, number_of_sensor);
gyro_mag = sqrt(lpms_GyroX.*lpms_GyroX + lpms_GyroY.*lpms_GyroY + lpms_GyroZ.*lpms_GyroZ);
gyro_magFilt_HP = zeros(maxFrame, number_of_sensor);
gyro_magFilt_LP = zeros(maxFrame, number_of_sensor);
gyro_magFilt = zeros(maxFrame, number_of_sensor);
Acc_g = zeros(maxFrame, 3, number_of_sensor);
Gyro_g = zeros(maxFrame, 3, number_of_sensor);
Quat = zeros(maxFrame, 4, number_of_sensor); % q_SE
LinAcc_g = zeros(maxFrame, 3, number_of_sensor);
lpms_LinAcc_g = zeros(maxFrame, 3, number_of_sensor);
Vel_g = zeros(maxFrame, 3, number_of_sensor);
lpms_Vel = zeros(maxFrame, 3, number_of_sensor);
Pos_g = zeros(maxFrame, 3, number_of_sensor);
lpms_Pos = zeros(maxFrame, 3, number_of_sensor);

% state definition : FC -0-> FO -1->
state = zeros(1, number_of_sensor);
stationary = ones(maxFrame, number_of_sensor);

% WIP event tick
FO_tick = ones(1, number_of_sensor); FC_tick = ones(1, number_of_sensor);

% LP filter coeffiecients
filtCutOff = 5;
[b_LP, a_LP] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');

% filter threshold
acc_mag_threshold = 0.04; % g
gyro_mag_threshold = 30; % dps

% choose which AHRS algorithm to use
AHRSalgorithm_cell = cell(1,number_of_sensor);
for j = 1:number_of_sensor
  % AHRSalgorithm_cell{j} = AHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'KpInit', 1);
  % AHRSalgorithm_cell{j} = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);
  % AHRSalgorithm_cell{j} = MadgwickAHRS('SamplePeriod', samplePeriod, 'Beta', 1);
  AHRSalgorithm_cell{j} = MadgwickAHRS('SamplePeriod', samplePeriod, 'Beta', 0.1);
end

% velocity/position drift compensation
VelDrift_flag = zeros(number_of_sensor);
VelDriftRate = zeros(number_of_sensor,3);
lpms_VelDriftRate = zeros(number_of_sensor,3);
PosDrift_flag = zeros(number_of_sensor);
PosDriftRate = zeros(number_of_sensor,3);
lpms_PosDriftRate = zeros(number_of_sensor,3);

% -------------------------------------------------------------------------
% Loop
% Initial convergence for AHRS algorithm
for j = 1:number_of_sensor
  for i = 1:1000
    AHRSalgorithm_cell{j}.UpdateIMU([0 0 0], [lpms_AccX(1,j) lpms_AccY(1,j) lpms_AccZ(1,j)]);
  end
  % Quat(1,:,j) = AHRSalgorithm_cell{j}.Quaternion;               % for AHRS
  Quat(1,:,j) = quaternConj(AHRSalgorithm_cell{j}.Quaternion);  % for MadgwickAHRS, MahonyAHRS
end

for i = 2:maxFrame
  % Detect WIP event
  % HP Filter
  acc_magFilt_HP(i,:) = acc_mag(i,:) - 1;
  acc_magFilt_HP(i,:) = abs(acc_magFilt_HP(i,:));
  gyro_magFilt_HP(i,:) = gyro_mag(i,:);
  % LP Filter
  acc_magFilt_LP(i,:) = b_LP(1)*acc_magFilt_HP(i,:) + b_LP(2)*acc_magFilt_HP(i - 1,:) - a_LP(2)*acc_magFilt_LP(i - 1,:);
  acc_magFilt(i,:) = acc_magFilt_LP(i,:);
  gyro_magFilt_LP(i,:) = b_LP(1)*gyro_magFilt_HP(i, :) + b_LP(2)*gyro_magFilt_HP(i-1, :) - a_LP(2)*gyro_magFilt_LP(i-1, :);
  gyro_magFilt(i,:) = gyro_magFilt_LP(i,:);

  for j = 1:number_of_sensor
    % Threshold detection
    if(acc_magFilt(i,j) < acc_mag_threshold && gyro_magFilt(i,j) < gyro_mag_threshold)
      stationary(i,j) = 1;
      if(state(j) == 1)
        state(j) = 0;
        FC_time(FC_tick(j), j) = lpms_TimeStamp(i, j);
        FC_tick(j) = FC_tick(j) + 1;
      end
    else
      stationary(i,j) = 0;
      if(state(j) == 0)
        state(j) = 1;
        FO_time(FO_tick(j), j) = lpms_TimeStamp(i, j);
        FO_tick(j) = FO_tick(j)+ 1;
      end
    end

    % Compute orientation from AHRS algorithm
    AHRSalgorithm_cell{j}.UpdateIMU(deg2rad([lpms_GyroX(i,j) lpms_GyroY(i,j) lpms_GyroZ(i,j)]), ...
    [lpms_AccX(i,j) lpms_AccY(i,j) lpms_AccZ(i,j)]);
    % Quat(i,:,j) = AHRSalgorithm_cell{j}.Quaternion;               % for AHRS
    Quat(i,:,j) = quaternConj(AHRSalgorithm_cell{j}.Quaternion);  % for MadgwickAHRS, MahonyAHRS
    % raw sensor data represented in global frame, x_e = q_ES*x_s*q_SE
    Acc_g(i,:,j) = quaternRotate([lpms_AccX(i,j) lpms_AccY(i,j) lpms_AccZ(i,j)], quaternConj(Quat(i,:,j)));
    LinAcc_g(i,:,j) = Acc_g(i,:,j) - [0 0 1]; % gravity compensation
    lpms_LinAcc_g(i,:,j) = quaternRotate([lpms_LinAccX(i,j) lpms_LinAccY(i,j) lpms_LinAccZ(i,j)], quaternConj(lpms_Quat(i,:,j)));
    Gyro_g(i,:,j) = quaternRotate([lpms_GyroX(i,j) lpms_GyroY(i,j) lpms_GyroZ(i,j)], quaternConj(Quat(i,:,j)));

    % Track position
    if(j < 3) % track left foot & right foot only
      % without velocity drift compensation
      % Vel_g(i,:,j) = Vel_g(i-1,:,j) + 0.5*(LinAcc_g(i-1,:,j) + LinAcc_g(i,:,j))*9.81*samplePeriod;
      % lpms_Vel(i,:,j) = lpms_Vel(i-1,:,j) + 0.5*(lpms_LinAcc_g(i-1,:,j) + lpms_LinAcc_g(i,:,j))*9.81*samplePeriod;
      % with velocity drift compensation
      Vel_g(i,:,j) = Vel_g(i-1,:,j) + 0.5*(LinAcc_g(i-1,:,j) + LinAcc_g(i,:,j))*9.81*samplePeriod - VelDriftRate(j,:)*samplePeriod;
      lpms_Vel(i,:,j) = lpms_Vel(i-1,:,j) + 0.5*(lpms_LinAcc_g(i-1,:,j) + lpms_LinAcc_g(i,:,j))*9.81*samplePeriod - lpms_VelDriftRate(j,:)*samplePeriod;

      % Compute velocity drift rate
      if(stationary(i,j) == 1)
        if(VelDrift_flag(j) == 1)
          VelDrift_flag(j) = 0;
          time_interval = lpms_TimeStamp(i,j) - FO_time(FO_tick(j)-1,j);
          VelDriftRate(j,:) = VelDriftRate(j,:) + Vel_g(i,:,j)/time_interval;
          lpms_VelDriftRate(j,:) = lpms_VelDriftRate(j,:) + lpms_Vel(i,:,j)/time_interval;
        end
        Vel_g(i,:,j) = [0 0 0];
        lpms_Vel(i,:,j) = [0 0 0];
      else
        VelDrift_flag(j) = 1;
      end

      % without position drift compensation + bounded z
      Pos_g(i,:,j) = Pos_g(i-1,:,j) + 0.5*(Vel_g(i-1,:,j) + Vel_g(i,:,j))*samplePeriod;
      lpms_Pos(i,:,j) = lpms_Pos(i-1,:,j) + 0.5*(lpms_Vel(i-1,:,j) + lpms_Vel(i,:,j))*samplePeriod;
      if(Pos_g(i,3,j) < 0)
        Pos_g(i,3,j) = 0;
      end
      if(lpms_Pos(i,3,j) < 0)
        lpms_Pos(i,3,j) = 0;
      end
      % with position drift compensation (z component only)
      % Pos_g(i,:,j) = Pos_g(i-1,:,j) + 0.5*(Vel_g(i-1,:,j) + Vel_g(i,:,j))*samplePeriod - PosDriftRate(j,:)*samplePeriod;
      % lpms_Pos(i,:,j) = lpms_Pos(i-1,:,j) + 0.5*(lpms_Vel(i-1,:,j) + lpms_Vel(i,:,j))*samplePeriod - lpms_PosDriftRate(j,:)*samplePeriod;
      %
      % % Compute position drift rate (z component only)
      % if(stationary(i,j) == 1)
      %   if(PosDrift_flag(j) == 1)
      %     PosDrift_flag(j) = 0;
      %     time_interval = lpms_TimeStamp(i,j) - FO_time(FO_tick(j)-1,j);
      %     PosDriftRate(j,:) = PosDriftRate(j,:) + Pos_g(i,:,j)/time_interval;
      %     PosDriftRate(j,1:2) = [0 0];
      %     lpms_PosDriftRate(j,:) = lpms_PosDriftRate(j,:) + lpms_Pos(i,:,j)/time_interval;
      %     lpms_PosDriftRate(j,1:2) = [0 0];
      %   end
      %   Pos_g(i,3,j) = 0;
      %   lpms_Pos(i,3,j) = 0;
      % else
      %   PosDrift_flag(j) = 1;
      % end

    end

  end

end

% Reorient quaternion
for j = 1:number_of_sensor
  Quat0 = quaternConj(Quat(1,:,j)); % q{E}->{S0}
  lpms_Quat0 = quaternConj(lpms_Quat(1,:,j)); % q{E}->{S0}
  for i = 1:maxFrame
    % quaternion with respect to foot frame q{S}->{S0} = q{E}->{S0} * q{S}->{E}
    Quat(i,:,j) = quaternProd(Quat0, Quat(i,:,j));
    lpms_Quat(i,:,j) = quaternProd(lpms_Quat0, lpms_Quat(i,:,j));
  end
end

% -------------------------------------------------------------------------
% 6DOF Animation
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% Compare AHRS algorithm
% Fix position
% posPlot1 = zeros(maxFrame, 3);
% posPlot2 = zeros(maxFrame, 3);
% posPlot2(:,1) = ones(maxFrame, 1);
% posPlot3 = 0.5*ones(maxFrame, 3);
% posPlot3(:,2) = zeros(maxFrame, 1);
% Plot position from lpms_LinAcc
% posPlot1 = lpms_Pos(:,:,1);
% posPlot2 = lpms_Pos(:,:,2);
% posPlot3 = lpms_Pos(:,:,3);
% Plot position from Acc_g
posPlot1 = Pos_g(:,:,1);
posPlot2 = Pos_g(:,:,2);
posPlot3 = Pos_g(:,:,3);

% Plot quaternion from LPMS-B2
% rotPlot1 = quatern2rotMat(lpms_Quat(:,:,1));
% rotPlot2 = quatern2rotMat(lpms_Quat(:,:,2));
% rotPlot3 = quatern2rotMat(lpms_Quat(:,:,3));
% Plot quaternion from AHRS/MahonyAHRS/MadgwickAHRS
rotPlot1 = quatern2rotMat(Quat(:,:,1));
rotPlot2 = quatern2rotMat(Quat(:,:,2));
rotPlot3 = quatern2rotMat(Quat(:,:,3));
% Plot relative quaternion (lpms_Quat & Quat)
% q_rel_1 = quaternProd(lpms_Quat(:,:,1), quaternConj(Quat(:,:,1)));
% q_rel_2 = quaternProd(lpms_Quat(:,:,2), quaternConj(Quat(:,:,2)));
% q_rel_3 = quaternProd(lpms_Quat(:,:,3), quaternConj(Quat(:,:,3)));
% rotPlot1 = quatern2rotMat(q_rel_1);
% rotPlot2 = quatern2rotMat(q_rel_2);
% rotPlot3 = quatern2rotMat(q_rel_3);

% Animation with spin
% SamplePlotFreq = 4;
% Spin = 100;
% n_Animation(3, posPlot1, rotPlot1, posPlot2, rotPlot2, posPlot3, rotPlot3, ...
%                 'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
%                 'Position', [9 39 1280 768], 'View', [(100:(Spin/(maxFrame-1)):(100+Spin))', 10*ones(maxFrame, 1)], ...
%                 'AxisLength', 0.1, 'ShowArrowHead', false, 'MP4fileName', 'WIP_6DOF_Animation', ...
%                 'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
%                 'CreateMP4', false, 'MP4fileNameEnum', true, 'MP4fps', ((1/samplePeriod) / SamplePlotFreq));
% Animation without spin
SamplePlotFreq = 4;
n_Animation(1, posPlot2, rotPlot2, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'DotsOnly', ...
                'Position', [9 39 1280 768], 'View', [135*ones(maxFrame, 1), 35*ones(maxFrame, 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, 'MP4fileName', 'WIP_6DOF_Animation', ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateMP4', false, 'MP4fileNameEnum', true, 'MP4fps', ((1/samplePeriod) / SamplePlotFreq));

% -------------------------------------------------------------------------
% Plot result
% -------------------------------------------------------------------------

% % -------------------------------------------------------------------------
% % Plot magnitude of acceleration/angular velocity with stationary detection
% number_of_subplot = 4;
%
% for plotSensor = 1:number_of_sensor
%   figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', ['Sensor Data : Gyrosope, Accelerometer, SensorId = ',num2str(plotSensor)]);
%   ax(1) = subplot(number_of_subplot,1,1);
%       hold on;
%       plot(lpms_TimeStamp(:,plotSensor), gyro_mag(:,plotSensor));
%       plot(lpms_TimeStamp(:,plotSensor), 100*stationary(:,plotSensor), 'k', 'LineWidth', 2);
%       title('Gyroscope (magnitude)');
%       xlabel('Time (s)');
%       ylabel('Angular velocity (^\circ/s)');
%       vline(FC_time(:,plotSensor), ':k');
%       vline(FO_time(:,plotSensor), ':r');
%       hold off;
%   ax(2) = subplot(number_of_subplot,1,2);
%       hold on;
%       plot(lpms_TimeStamp(:,plotSensor), acc_mag(:,plotSensor));
%       plot(lpms_TimeStamp(:,plotSensor), stationary(:,plotSensor), 'k', 'LineWidth', 2);
%       title('Accelerometer (magnitude)');
%       xlabel('Time (s)');
%       ylabel('Acceleration (g)');
%       vline(FC_time(:,plotSensor), ':k');
%       vline(FO_time(:,plotSensor), ':r');
%       hold off;
%   linkaxes(ax,'x');
%   delta_angle = rad2deg(acos(abs(Quat(:,1,plotSensor)))*2); % total angle
%   [phi_z, phi_xy] = quaternSplit(quaternConj(Quat(:,:,plotSensor))); % angle by component (q{S0}->{S})
%   euler = quatern2euler(quaternConj(Quat(:,:,plotSensor)));
%   delta_angle(1) = 0;
%   phi_z(1) = 0; phi_xy(1) = 0;
%   ax(3) = subplot(number_of_subplot,1,3);
%       hold on;
%       plot(lpms_TimeStamp(:,plotSensor), delta_angle);
%       plot(lpms_TimeStamp(:,plotSensor), rad2deg(phi_z));
%       plot(lpms_TimeStamp(:,plotSensor), rad2deg(phi_xy));
%       plot(lpms_TimeStamp(:,plotSensor), stationary(:,plotSensor), 'k', 'LineWidth', 2);
%       title('Quaternion (Madgwick)');
%       xlabel('Time (s)');
%       ylabel('Deviation Angle (degrees)');
%       vline(FC_time(:,plotSensor), ':k');
%       vline(FO_time(:,plotSensor), ':r');
%       hold off;
%   linkaxes(ax,'x');
%   ax(4) = subplot(number_of_subplot,1,4);
%       hold on;
%       plot(lpms_TimeStamp(:,plotSensor), delta_angle);
%       plot(lpms_TimeStamp(:,plotSensor), rad2deg(euler(:,1)));
%       plot(lpms_TimeStamp(:,plotSensor), rad2deg(euler(:,2)));
%       plot(lpms_TimeStamp(:,plotSensor), rad2deg(euler(:,3)));
%       plot(lpms_TimeStamp(:,plotSensor), stationary(:,plotSensor), 'k', 'LineWidth', 2);
%       title('Quaternion (Madgwick)');
%       xlabel('Time (s)');
%       ylabel('Deviation Angle (degrees)');
%       vline(FC_time(:,plotSensor), ':k');
%       vline(FO_time(:,plotSensor), ':r');
%       hold off;
%   linkaxes(ax,'x');
% %   delta_angle = rad2deg(acos(abs(lpms_Quat(:,1,plotSensor)))*2); % total angle
% %   [phi_z, phi_xy] = quaternSplit(quaternConj(Quat(:,:,plotSensor))); % angle by component
% %   delta_angle(1) = 0;
% %   phi_z(1) = 0; phi_xy(1) = 0;
% %   ax(4) = subplot(number_of_subplot,1,4);
% %       hold on;
% %       plot(lpms_TimeStamp(:,plotSensor), delta_angle);
% %       plot(lpms_TimeStamp(:,plotSensor), rad2deg(phi_z));
% %       plot(lpms_TimeStamp(:,plotSensor), rad2deg(phi_xy));
% %       plot(lpms_TimeStamp(:,plotSensor), stationary(:,plotSensor), 'k', 'LineWidth', 2);
% %       title('Quaternion (LPMS)');
% %       xlabel('Time (s)');
% %       ylabel('Deviation Angle (degrees)');
% %       vline(FC_time(:,plotSensor), ':k');
% %       vline(FO_time(:,plotSensor), ':r');
% %       hold off;
% %   linkaxes(ax,'x');
% end
