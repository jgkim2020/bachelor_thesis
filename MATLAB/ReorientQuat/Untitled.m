%%
clear;
close all;
clc;
addpath('..\lib\hline_vline');
addpath('..\lib\Quaternions');
addpath('..\lib\AHRS');
addpath('..\lib\Mahony_AHRS');
addpath('..\lib\Madgwick_AHRS');
addpath('..\lib\SixDofAnimation');

%% test1
filepath = '..\Datasets\';
filename = '0914';
csvdata = csvread([filepath,filename,'.csv'], 1, 0);
Quat = csvdata(:,16:19);

Quat0 = Quat(1,:);
for i = 1:length(Quat)
    Quat_(i,:) = quaternProd(Quat(i,:), quaternConj(Quat0)); % expected
    Quat__(i,:) = quaternProd(quaternConj(Quat0), Quat(i,:)); % ???!
    Quat___(i,:) = quaternProd(quaternConj(Quat(i,:)), Quat0);
end

%% test2
clear;
close all;
clc;

for i = 1:100
    Quat(i,:) = [cos(0.5*pi/200*i) -sin(0.5*pi/200*i)*[1/sqrt(14) 2/sqrt(14) 3/sqrt(14)]];
end

Quat0 = Quat(50,:);
for i = 1:100
    Quat_(i,:) = quaternProd(Quat(i,:), quaternConj(Quat0)); % expected
    Quat__(i,:) = quaternProd(quaternConj(Quat0), Quat(i,:));
end

%% test3
AHRS = MadgwickAHRS('SamplePeriod', 0.01, 'Beta', 0.1);
for i = 1:10000
    AHRS.UpdateIMU([0 0 0], [-0.3162 0.609 0.7267]);
end
Quat = AHRS.Quaternion()
