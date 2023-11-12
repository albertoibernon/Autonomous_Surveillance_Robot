clear; close all; clc;

load('output\time_calibration.mat','time')
load('output\pos_real_calibration.mat','pos_real')
load('output\odometry_calibration.mat','odometry_mes')

error_x = odometry_mes(:,1) - pos_real(:,1);
error_y = odometry_mes(:,2) - pos_real(:,2);

figure();plot(time,error_x);hold on; grid on;plot(time,error_y);legend('error x','error y')


