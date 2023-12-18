close all; clear; clc;

% Cargar Biblioteca Apolo/Matlab
addpath(genpath('C:\Program Files\Apolo'))

% Configuración
global time_i
robot = 'Marvin';
laser = 'LMS100';

%% Inicialización
pose_init = [1, 1, 0]; % Pose inicial
pose_goal = [8, 1, 0];     % Pose objetivo
pos_init = init(robot,pose_init);

time_step       = 0.05;
simulation_time = 75;
giro=0;
mapa = importdata('mapa.xlsx');

% Definimos las varianzas
Pxini = 0.01;
Pyini = 0.01;
Pthetaini = 0.01;
Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

% Varianza del ruido del proceso
QL=1.0312e-05;
QG=1.955e-05;
Qk_1 =[QL 0;0 QG];

% Varianza del lidar
% R = 0.01;
R = 9.62014799313916e-06;

% Inicialización
time = []; pos_real = []; odometry_mes = []; lidar_mes = []; odometry_cor=[]; P_acumulado = []; C_acumulado=[];
X_pre=[];
muestra=[0.02; 0.02; 0.02; 0.02; 0.02; 0.02; 0.02; 0.02];
correccion=[0;0;0];
Xk     = [pos_init(1) pos_init(2) pos_init(4)]'; % Real initial position
Xk_est = Xk;                                    % Estimated initial position
angulo_scattering = linspace(-0.75*pi,0.75*pi,539) - 1.5*pi/539*ones(1,539);
e_acumulado=[];

vel_avance  = 1; % Avance [m/s]
vel_rot_max = 0.5; % Avance [m/s]
[pthObj,vel_comando] = planificador_global(pose_init,pose_goal,time_step,simulation_time);
ii = 1;
%% Bucle de Simulación
while true
    %% Planificador
    % speeds = planificador; % [velocidad de avance, velocidad de rotación] [m/s,rad/s]
    speeds = vel_comando(ii,:);
    %% DKE
    pos_real_i = dke(robot,speeds,time_step); % [x,y,z,theta] [m,rad]

    %% Odometría (Sensores Propioceptivos)
    odometry_mes_i = apoloGetOdometry(robot); % [x,y,theta] [m,rad]
    
    %% Percepción (Sensores Exteroceptivos)
    lidar_mes_i = apoloGetLaserData(laser);
    b=size(lidar_mes_i);
    coords_polar = [lidar_mes_i;angulo_scattering];

    % Generar Nube de Puntos en Coord Cartesinas en Ejes Cuerpo del Robot
    coords_cart=polaresACartesianas(coords_polar);

    % % Plotear Nube de Puntos
    % coordenadasX = coords_cart(1,:) + odometry_mes_i(1); % Para fijar eje:
    % coordenadasY = coords_cart(2,:) + odometry_mes_i(2); % Para fijar eje:
    % figure();plot(coordenadasX, coordenadasY, 'o'); grid minor;

    %% Navegación
    % Nuevo ciclo (k-1 = k)
    Uk = speeds*time_step; % [distancia ángulo]
    Xk_1 = Xk;
    Pk_1 = Pk;
    
    % Prediccion del estado
    X_k = [(Xk_1(1) + Uk(1)*cos(Xk_1(3)+(Uk(2)/2)));
           (Xk_1(2) + Uk(1)*sin(Xk_1(3)+(Uk(2)/2)));
           (Xk_1(3) + Uk(2))];

    Ak = [1 0 (-Uk(1)*sin(Xk_1(3)+Uk(2)/2));
          0 1 (Uk(1)*cos(Xk_1(3)+Uk(2)/2));
          0 0 1                             ];
    Bk = [(cos(Xk_1(3)+Uk(2)/2)) (-0.5*Uk(1)*sin(Xk_1(3)+Uk(2)/2));
          (sin(Xk_1(3)+Uk(2)/2)) (0.5*Uk(1)*cos(Xk_1(3)+Uk(2)/2));
           0                     1                                 ];
    P_k = Ak*Pk_1*((Ak)') + Bk*Qk_1*((Bk)');

    % Predicción del vector de las observaciones
    [dist_est,ang_est,points_est] = prediccion_observacion(mapa,[X_k(1) X_k(2)]);

    % Observación: vector de medidas
    distancia_medida = observacion(coords_cart');

    % Comparación e Innovación (matches)
    [matches,H] = comparacion(distancia_medida,[X_k(1) X_k(2)],dist_est,points_est);
    
    if isempty(matches)
        Xk = X_k;
        Pk = P_k;
    else
        a=size(H);
        Rk=eye(a(1))*(R);
        Sk = H*P_k*((H)') + Rk;

        % Distancia de Mahalonobis
        Yk=[]; Hk=[];
        for j=1:length(matches)
            dist_mahalanobis = matches(j)*inv(Sk)*matches(j);
            if dist_mahalanobis(j,j) < 0.0039 % Parámetro extraído de la tabla de Mahalanobis para n_z = 1
                Yk = [Yk; matches(j)]; % Innovación
                Hk = cat(1,Hk,H(j,:));
            end
        end
        
        a=size(Hk);
        if a==0
            Xk = X_k;
            Pk = P_k;
        else
            Rk = eye(a(1))*(R);
            Sk = Hk*P_k*(Hk') + Rk;
            Wk = P_k*(Hk')*inv(Sk);
    
            % Correccion
            Xk = X_k + Wk*Yk;
            Pk = (eye(3) - Wk*Hk)*P_k;
            correccion= Wk*Yk;
        end
    end
    P_valores=[Pk(1,1),Pk(2,2),Pk(3,3)];
    e = [Xk(1)-pos_real_i(1);
         Xk(2)-pos_real_i(2);
         Xk(3)-pos_real_i(4)];
    e_acumulado = [e_acumulado; e'];

    %% Almacenar variables
    [time,pos_real,odometry_mes,odometry_cor,P_acumulado,C_acumulado] = acumular(time,pos_real,odometry_mes,odometry_cor,P_acumulado, C_acumulado,pos_real_i,odometry_mes_i,Xk',P_valores,correccion');

    %% Actualizar bucle
    apoloUpdate();
    apoloResetOdometry(robot,Xk');
    pause(time_step/2);
    setTime(time_step);
    ii = ii + 1;
    if time_i > simulation_time
        break;
    end
end

%plot
figure()
plot(time,pos_real(:,1)); 
hold on;
grid on;
plot(time,odometry_cor(:,1));
plot(time,odometry_mes(:,1));
xlabel('time (s)');
ylabel('x (m)');
title('x');
legend('real','ekf','odometry')

% figure()
% plot(time,P_acumulado(:,1)); 
% hold on;
% grid on;
% plot(time,P_acumulado(:,2)); 
% plot(time,P_acumulado(:,3)); 
% legend('Px','Py','Ptheta')

figure()
plot(time,e_acumulado(:,1)); 
hold on;
grid on;
plot(time,e_acumulado(:,2)); 
%plot(time,e_acumulado(:,3)); 
legend('ex','ey','etheta')

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
