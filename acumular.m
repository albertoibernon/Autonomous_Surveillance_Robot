function [time,pos_real,odometry_mes,odometry_cor,P_acumulado,C_acumulado] = acumular(time,pos_real,odometry_mes,odometry_cor,P_acumulado, C_acumulado,pos_real_i,odometry_mes_i,Xk,Pk,correccion)
global time_i
time         = [time; time_i];
pos_real     = [pos_real; pos_real_i];
odometry_mes = [odometry_mes; odometry_mes_i];
odometry_cor    = [odometry_cor; Xk];
P_acumulado    = [P_acumulado; Pk];
C_acumulado = [C_acumulado;correccion];
end