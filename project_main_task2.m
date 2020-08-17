% main code here
clc
clear all
close all

startup();
showIP();

[xhat, meas] = filterTemplate();

% [xhat, meas] = filterTemplate_copy(meas.acc, meas.gyr, meas.mag);

%% task 2
% load steady data [not necessary to send input]
load meas-Flat.mat
% load  S7_steady

% compute the mean and covariance
mu_acc = mean(meas.acc(:, ~any(isnan(meas.acc),1)), 2);
mu_gyr = mean(meas.gyr(:, ~any(isnan(meas.gyr),1)), 2);
mu_mag = mean(meas.mag(:, ~any(isnan(meas.mag),1)), 2);

for i=1:3 % for 3 dimension
    cov_acc(i) = cov(meas.acc(i,~any(isnan(meas.acc),1)));
    cov_gyr(i) = cov(meas.gyr(i,~any(isnan(meas.gyr),1)));
    cov_mag(i) = cov(meas.mag(i,~any(isnan(meas.mag),1)));
end

Mean = [mu_acc mu_gyr mu_mag];% each column is the mean and covariance of each sensor
Cov = [ cov_acc' cov_gyr' cov_mag'];

disp(['The mean matrix of sensor is:']);
disp(Mean);
disp(['The covariance matrix of sensor is:']);
disp(Cov);

% plot histogram
bins = 100;
meas_set = {meas.acc, meas.gyr, meas.mag};
mu_set = {mu_acc, mu_gyr, mu_mag};
cov_set = {cov_acc, cov_gyr, cov_mag};
sensor_set = {'accelator', 'gyroscope', 'magnetometer'};
axes = ['x', 'y', 'z'];
color = colormap(lines);

% histograms of measurements for some sensors and axes
% for-loop: 3 types of sensors
for i = 1:3
    % for-loop: 3 axes [x, y, z]
    figure(i);    
    for j = 1:3
        subplot(3,1,j); 
        histogram(meas_set{i}(j,:), bins,'Normalization','count','FaceAlpha',1,'FaceColor',color(j,:));
        title(['measurement histograms for ', sensor_set{i}, ' sensor']);
        legend([axes(j), 'axis'])
    end
end

% plot signals over time  
%%%%%%%%%%%%% insert the time and axis for different %%%%%%%%%%%%%%%%%%%%%%
% sensor
figure;
% for-loop sor 3 axes: [x, y, z]

for i = 1:3
    % for-loop: 3 axes [x, y, z]
    figure(i+3);
    
    for j = 1:3
        subplot(3,1,j); 
        plot(meas.t, meas_set{i}(j,:),'Color',color(j,:));
        title(['Signals over time for ', sensor_set{i}, ' sensor']);
        legend([axes(j), 'axis']);
        
    end
end


%% task 3




