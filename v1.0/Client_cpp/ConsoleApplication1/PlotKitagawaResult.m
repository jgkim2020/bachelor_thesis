clear;
close all;
clc;

filepath = '.\';
filename = 'Kitagawa_result_left';
csvdata1 = csvread([filepath,filename,'.csv'], 1, 0);
filepath = '.\';
filename = 'Kitagawa_result_right';
csvdata2 = csvread([filepath,filename,'.csv'], 1, 0);

number_of_sensor = 1;
gait_sequence1 = csvdata1(:,1);
gait_sequence2 = csvdata2(:,1);

% -------------------------------------------------------------------------
% Plot result
for plotSensor = 1:number_of_sensor
  figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', ['Foot trajectory : SensorId = ','left']);
  hold on;
  for i = 1:max(gait_sequence1)
      t_scope = find(gait_sequence1 == i);
      transversePos1 = sqrt(csvdata1(t_scope,2).^2 + csvdata1(t_scope,3).^2);
      verticalPos1 = csvdata1(t_scope,4);
      distance1(i) = transversePos1(length(t_scope));
      distancex1(i) = csvdata1(t_scope(length(t_scope)),2);
      distancey1(i) = csvdata1(t_scope(length(t_scope)),3);
      plot(transversePos1,verticalPos1);
  end
  title('Foot Trajectory (left)');
  xlabel('Transverse distance (m)');
  ylabel('Vertical distance (m)');
  hold off;
  figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', ['Foot trajectory : SensorId = ','right']);
  hold on;
  for i = 1:max(gait_sequence2)
      t_scope = find(gait_sequence2 == i);
      transversePos2 = sqrt(csvdata2(t_scope,2).^2 + csvdata2(t_scope,3).^2);
      verticalPos2 = csvdata2(t_scope,4);
      distance2(i) = transversePos2(length(t_scope));
      distancex2(i) = csvdata2(t_scope(length(t_scope)),2);
      distancey2(i) = csvdata2(t_scope(length(t_scope)),3);
      plot(transversePos2,verticalPos2);
  end
  title('Foot Trajectory (right)');
  xlabel('Transverse distance (m)');
  ylabel('Vertical distance (m)');
  hold off;
end
