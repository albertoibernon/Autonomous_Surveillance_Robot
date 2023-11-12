function [time,pos_real,odometry_mes,lidar_mes] = acumular(time,pos_real,odometry_mes,lidar_mes,pos_real_i,odometry_mes_i,lidar_mes_i)
global time_i
time         = [time; time_i];
pos_real     = [pos_real; pos_real_i];
odometry_mes = [odometry_mes; odometry_mes_i];
lidar_mes    = [lidar_mes; lidar_mes_i];

end