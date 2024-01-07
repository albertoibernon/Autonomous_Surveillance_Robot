close all; clear; clc;

% Cargar Biblioteca Apolo/Matlab
addpath(genpath('C:\Program Files\Apolo'))

% Configuración
global integralState
robot   = 'Marvin';
laser   = 'LMS100';
mapa    = importdata('mapa.xlsx');
balizas = importdata('mapa_balizas.xlsx');

%% Inicialización
pose_init = [1, 1, 0];  % Pose inicial  [x, y, orientación] [m, m, rad]
pose_goal = [2, 27, 0]; % Pose objetivo [x, y, orientación] [m, m, rad]
time_step = 0.1;        % [s]
pos_init  = init(robot,pose_init);

% Varianza del ruido del proceso
QL   = 1.0312e-05;
QG   = 1.955e-05;
Qk_1 = [QL 0;0 QG];

% Varianza del ruido del lidar
R     = 9.62014799313916e-02;
vdist = 1.4481e-04;
vang  = 4.7432e-04;

% Matriz de varianzas de la estimación
Pxini     = 0.01;
Pyini     = 0.01;
Pthetaini = 0.01;
Pk        = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

% EKF
Xk         = [pos_init(1) pos_init(2) pos_init(4)]'; % Estimación posición inicial
correccion = [0;0;0];

% Variables para almacenar
time = []; pos_real = []; odometry_mes = []; lidar_mes = []; Xk_acumulado=Xk'; 
P_acumulado = []; C_acumulado=[]; e_acumulado=[]; comando_acu = []; up_acu = []; 
ui_acu = []; ud_acu = []; error_pos_acu = []; dist_sonic_left_acu = []; 
dist_sonic_middle_acu = []; dist_sonic_right_acu = [];

% Planificador Global
[pathObj,planner] = planificador_global(pose_init,pose_goal);

% Planificador Local
jj = 2;
planificador_local_flag = 0;

%% Bucle de Simulación
while true

    if planificador_local_flag == 0 % Para ejecutar la planificación local
        % Finalizar simulación al completar la lista de waypoints de la planificación global
        if jj > length(pathObj.States(:,1))
            break;
        end
                
        %% Planificador Local y Control Activo
        [vel_comando, pthObj] = planificador_local(planner,Xk',pathObj.States(jj,:),time_step);
        comando = vel_comando(1,:);
        ii = 1; planificador_local_flag = 1; obstaculo_flag = 0; integralState = 0;
        up = 0; ui = 0; ud = 0; error_pos = 0; dist_sonic = [0 0 0]; 

    else % Cuando ya hay una planificación local cargada
        %% Detección de Obstáculos y Control Reactivo
        [comando,up,ui,ud,error_pos,dist_sonic,ii] = control_reactivo(vel_comando(ii,:),...
         pthObj.States(ii+1,:),Xk_acumulado(end-1,:),Xk_acumulado(end,:),vel_comando,obstaculo_flag,ii);  
    end

    %% DKE (movimiento real del robot)
    pos_real_i = dke(robot,comando,time_step); % [x,y,z,theta] [m,rad]

    %% Odometría (Sensores Propioceptivos)
    odometry_mes_i = apoloGetOdometry(robot); % [x,y,theta] [m,rad]

    %% Navegación (Extended Kalman Filter)
    [Xk,Pk,Vec] = navegacion(Xk,Pk,Qk_1,balizas,odometry_mes_i,vdist,vang);
    
    P_valores   = [Pk(1,1),Pk(2,2),Pk(3,3)];
    e           = [Xk(1)-pos_real_i(1);Xk(2)-pos_real_i(2);Xk(3)-pos_real_i(4)];
    e_acumulado = [e_acumulado; e'];

    % Plotear posición real (verde) y posición estimada (magenta)
    figure(1)
    plot(pos_real_i(1),pos_real_i(2),'g*',"MarkerSize",2)
    plot(Xk(1),Xk(2),'m*',"MarkerSize",2)

    %% Almacenar variables
    [time,pos_real,odometry_mes,Xk_acumulado,P_acumulado,C_acumulado,comando_acu,...
     up_acu,ui_acu,ud_acu,error_pos_acu,dist_sonic_left_acu,dist_sonic_middle_acu,...
     dist_sonic_right_acu] = acumular(time,pos_real,odometry_mes,Xk_acumulado,P_acumulado,...
     C_acumulado,pos_real_i,odometry_mes_i,Xk',P_valores,correccion',comando_acu,up_acu,ui_acu,...
     ud_acu,error_pos_acu,dist_sonic_left_acu,dist_sonic_middle_acu,dist_sonic_right_acu,...
     comando(2),up,ui,ud,error_pos,dist_sonic);
      
    %% Actualizar bucle
    apoloUpdate();
    apoloResetOdometry(robot,Xk');
    % pause(time_step/2);
    setTime(time_step);

    % Caso en el que se ha alcanzando el final de la planificación local y
    % no se ha detectado obstáculo -> Selección del siguiente waypoint
    if ii == length(vel_comando(:,1)) && obstaculo_flag == 0
        planificador_local_flag = 0;
        jj = jj + 1;
        obstaculo_flag = 0;

    % Caso en el que se ha alcanzando el final de la planificación local y
    % se ha detectado obstáculo -> Selección del mismo waypoint
    elseif ii == length(vel_comando(:,1)) && obstaculo_flag == 1
        planificador_local_flag = 0;
        obstaculo_flag = 0;

    % Caso en el que no se ha alcanzado el final de la planificación local
    % -> Siguiente comando del planificador local
    else
        ii = ii + 1;
    end
end

figure(1);
legend('RRT','Waypoint','Reference Pose','Real Pose','Estimated Pose')

%% Plotear gráficos
figure()
plot(time,dist_sonic_left_acu,'*');
hold on;
grid on;
plot(time,dist_sonic_middle_acu,'*'); 
plot(time,dist_sonic_right_acu,'*'); 
legend('ultra izquierdo','ultra centro','ultra derecho')
xlabel('Time (s)');ylabel('Distance (m)'); title('Ultrasonic sensor distance to walls and obstacles')

figure()
plot(time,comando_acu); 
hold on;
grid on;
plot(time,up_acu);
plot(time,ui_acu); 
plot(time,ud_acu); 
legend('Angular Velocity','Proportional','Integral','Derivative')
title('Controller Contributions: Ang Velocity Command')
xlabel('Time (s)');ylabel('Distance (m)');

figure()
plot(time,error_pos_acu); 
hold on;
grid on;
title('Absolute Position Error (Reference and Estimation)')
xlabel('Time (s)');ylabel('Distance (m)');

figure()
plot(pos_real(:,1),pos_real(:,2)); 
hold on;
grid on;
plot(Xk_acumulado(:,1),Xk_acumulado(:,2));
plot(odometry_mes(:,1),odometry_mes(:,2));
xlabel('x (m)');
ylabel('y (m)');
title('Trayectoria');
legend('real','ekf','odometry')

figure()
plot(time,P_acumulado(:,1)); 
hold on;
grid on;
plot(time,P_acumulado(:,2)); 
plot(time,P_acumulado(:,3)); 
legend('Px','Py','Ptheta')
title('EKF Covariance')
xlabel('Time (s)');

figure()
plot(time,e_acumulado(:,1)); 
hold on;
grid on;
plot(time,e_acumulado(:,2)); 
%plot(time,e_acumulado(:,3)); 
legend('ex','ey')
title('EKF Performance: Position Error (Real VS Estimation)')
xlabel('Time (s)');ylabel('Distance (m)');

% figure()
% plot(time,C_acumulado(:,1)); 
% hold on;
% grid on;
% plot(time,C_acumulado(:,2)); 
% plot(time,C_acumulado(:,3)); 
% legend('Cx','Cy','Ctheta')

%% Save outputs
% save('output\time_calibration.mat','time')
% save('output\pos_real_calibration.mat','pos_real')
% save('output\odometry_calibration.mat','odometry_mes')
% save('output\lidar_calibration.mat','lidar_mes')

%% Post-pro
% figure();plot(time,pos_real(:,1)); hold on;grid on;plot(time,odometry_mes(:,1));xlabel('time (s)');ylabel('x (m)');title('x');legend('real','measured')
% figure();plot(time,pos_real(:,2)); hold on;grid on;plot(time,odometry_mes(:,2));xlabel('time (s)');ylabel('y (m)');title('y');legend('real','measured')
% figure();plot(time,pos_real(:,4).*180/pi); hold on;grid on;plot(time,odometry_mes(:,3).*180/pi);xlabel('time (s)');ylabel('theta (deg)');title('theta');legend('real','measured')
% figure();plot(pos_real(:,1),pos_real(:,2)); hold on;grid on;plot(odometry_mes(:,1),odometry_mes(:,2));xlabel('x (m)');ylabel('y (m)');title('trayectoria');legend('real','measured')
