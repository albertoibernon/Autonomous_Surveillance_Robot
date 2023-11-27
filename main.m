close all; clear; clc;

% Cargar Biblioteca Apolo/Matlab
addpath(genpath('C:\Program Files\Apolo'))

% Configuración
global time_i
robot = 'Marvin';
laser = 'LMS100';

time_step       = 0.05;
simulation_time = 40;
giro=0;
mapa = importdata('mapa.xlsx');

% Definimos las varianzas
Pxini = 0.1;
Pyini = 0.1;
Pthetaini = 0.1;
Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

% Varianza del ruido del proceso
QL=1.0312e-05;
QG=1.955e-05;
Qk_1 =[QL 0;0 QG];

% Varianza del lidar
%R = 1.0125e-02;
R = 0.01;

% Inicialización
init(robot)
time = []; pos_real = []; odometry_mes = []; lidar_mes = []; odometry_cor=[]; P_acumulado = []; C_acumulado=[];
X_pre=[];
Xk_est=[1;1;0];
muestra=[0.02; 0.02; 0.02; 0.02; 0.02; 0.02; 0.02; 0.02];
location= apoloGetLocationMRobot(robot);
correccion=[0;0;0];
Xk=[location(1);location(2);location(4)];

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
    t = t*(((1.5*pi))/b(2));% t*(270º/539 puntos)
    % Para fijar eje:
    % giro=giro+odometry_mes_i(3);
    % t=t+giro;
    coords=cat(1,lidar_mes_i,t);
    % Generar el array de ángulos
    %lidar_mes_i
    cart=polaresACartesianas(coords);
    %Para fijar eje:
    % cart(1, :)=cart(1, :)+odometry_mes_i(1);
    % cart(2, :)=cart(2, :)+odometry_mes_i(2);
    % coordenadasX = cart(1, :)+odometry_mes_i(1);
    % coordenadasY = cart(2, :)+odometry_mes_i(2);
    
    % Plotear los puntos
    %plot(coordenadasX, coordenadasY, 'o');

    %% Nuevo ciclo, k-1 = k.
    
    Uk=speeds*time_step;
    Xk_est = [(Xk_est(1) + Uk(1)*cos(Xk_est(3)+(Uk(2)/2)));
           (Xk_est(2) + Uk(1)*sin(Xk_est(3)+(Uk(2)/2)));
           (Xk_est(3) + Uk(2))];

    X_pre=[X_pre;Xk_est'];
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
    %Observación, predicción e innoviación
    [match,Hk]=navegacion(X_k,cart',mapa,muestra);
    if isempty(match)
        Xk = X_k;
        Pk = P_k;
    else
        Yk=match(1,:);
        muestra=cat(1,muestra, match(1,:)');
        a=size(Hk);
        Rk=eye(a(1))*R;
        Sk = Hk*P_k*((Hk)') + Rk;
        Wk = P_k*((Hk)')*inv(Sk);
    
        % Correccion
        Xk = X_k + Wk*Yk';
        Pk = (eye(3)-Wk*Hk)*P_k;
        correccion= Wk*Yk';
    end
    P_valores=[Pk(1,1),Pk(2,2),Pk(3,3)];
    ex=Xk(1)-pos_real_i(1)
    ey=Xk(1)-pos_real_i(1)
    
    

    % %S�lo para almacenarlo
    % Xestimado(:,l) = Xk;
    % Pacumulado(1,l) = Pk(1,1);
    % Pacumulado(2,l) = Pk(2,2);
    % Pacumulado(3,l) = Pk(3,3);

    %% Almacenar variables
    [time,pos_real,odometry_mes,odometry_cor,P_acumulado,C_acumulado] = acumular(time,pos_real,odometry_mes,odometry_cor,P_acumulado, C_acumulado,pos_real_i,odometry_mes_i,Xk',P_valores,correccion');
    
    %% Actualizar bucle
    apoloUpdate();
    p=size(muestra);
    if p(1)>50
        while p(1)>50
            muestra(1,:)=[];
            p=size(muestra);
        end
    end
    % apoloResetOdometry(robot,pos_real_i(1:3));
    % apoloResetOdometry(robot,[0 0 0]);
    pause(time_step/2);
    setTime(time_step);
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
plot(time,X_pre(:,1));
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