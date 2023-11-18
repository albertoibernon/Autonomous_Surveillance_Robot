close all; clear; clc;

% Cargar Biblioteca Apolo/Matlab
addpath(genpath('C:\Program Files\Apolo'))

% Configuración
global time_i
robot = 'Marvin';
laser = 'LMS100';

time_step       = 5;
%% 
simulation_time = 30;

% Inicialización
init(robot)
time = []; pos_real = []; odometry_mes = []; lidar_mes = [];

% Bucle de Simulación
while true
    %% Planificador
    speeds = planificador; % [velocidad de avance, velocidad de rotación] [m/s,rad/s]
    
    %% DKE
    pos_real_i = dke(robot,speeds,time_step); % [x,y,z,theta] [m,rad]

    %% Odometría (Sensores Propioceptivos)
    odometry_mes_i = apoloGetOdometry(robot); % [x,y,theta] [m,rad]
    
    %% Percepción (Sensores Exteroceptivos)
    lidar_mes_i = apoloGetLaserData(laser);

    %% Navegación
    b=size(lidar_mes_i);
    t = 1:b(2);% t = 1:539
    t = t*(1.5*pi/b(2));% t*(270º/539)
    coords=cat(1,lidar_mes_i,t);
    % Generar el array de ángulos
    %lidar_mes_i
    cart=polaresACartesianas(coords);
    coordenadasX = cart(1, :);
    coordenadasY = cart(2, :);
    
    % Plotear los puntos
    %plot(coordenadasX, coordenadasY, 'o');
    navegacion(odometry_mes_i,cart');

    %% Almacenar variables
    [time,pos_real,odometry_mes,lidar_mes] = acumular(time,pos_real,odometry_mes,lidar_mes,pos_real_i,odometry_mes_i,lidar_mes_i);
    
    %% Actualizar bucle
    apoloUpdate();
    apoloResetOdometry(robot,pos_real_i(1:3));
    pause(time_step/2);
    setTime(time_step);
    if time_i > simulation_time
        break;
    end
end

%% Save outputs
save('output\time_calibration.mat','time')
save('output\pos_real_calibration.mat','pos_real')
save('output\odometry_calibration.mat','odometry_mes')
save('output\lidar_calibration.mat','lidar_mes')

%% Post-pro
figure();plot(time,pos_real(:,1)); hold on;grid on;plot(time,odometry_mes(:,1));xlabel('time (s)');ylabel('x (m)');title('x');legend('real','measured')
figure();plot(time,pos_real(:,2)); hold on;grid on;plot(time,odometry_mes(:,2));xlabel('time (s)');ylabel('y (m)');title('y');legend('real','measured')
figure();plot(time,pos_real(:,4).*180/pi); hold on;grid on;plot(time,odometry_mes(:,3).*180/pi);xlabel('time (s)');ylabel('theta (deg)');title('theta');legend('real','measured')
figure();plot(pos_real(:,1),pos_real(:,2)); hold on;grid on;plot(odometry_mes(:,1),odometry_mes(:,2));xlabel('x (m)');ylabel('y (m)');title('trayectoria');legend('real','measured')
