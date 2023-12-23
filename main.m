close all; clear; clc;

% Cargar Biblioteca Apolo/Matlab
addpath(genpath('C:\Program Files\Apolo'))

% Configuración
global time_i
robot = 'Marvin';
laser = 'LMS100';

time_step       = 0.05;
%time_step       = 0.5;
simulation_time =89;
giro=0;
mapa = importdata('mapa.xlsx');
balizas = importdata('mapa_balizas.xlsx');

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
R = 0.001;
vdist = 1.4481e-05;
vang = 4.7432e-05;

% Inicialización
init(robot)
time = []; pos_real = []; odometry_mes = []; lidar_mes = []; odometry_cor=[]; P_acumulado = []; C_acumulado=[];
X_pre=[];
muestra=[0.02; 0.02; 0.02; 0.02; 0.02; 0.02; 0.02; 0.02];
location= apoloGetLocationMRobot(robot);
correccion=[0;0;0];
Xk=[location(1);location(2);location(4)];
Xk_est=[location(1);location(2);location(4)];
e_acumulado=[];

% Bucle de Simulación
while true
    %% Planificador
    speeds = planificador; % [velocidad de avance, velocidad de rotación] [m/s,rad/s]
    
    %% DKE
    pos_real_i = dke(robot,speeds,time_step); % [x,y,z,theta] [m,rad]

    %% Odometría (Sensores Propioceptivos)
    odometry_mes_i = apoloGetOdometry(robot); % [x,y,theta] [m,rad]

    %% Nuevo ciclo, k-1 = k.
    
    Xk_1 = Xk;
    Pk_1 = Pk;

    Uk = [sqrt((Xk_1(1)-odometry_mes_i(1))^2+(Xk_1(2)-odometry_mes_i(2))^2);odometry_mes_i(3)-Xk_1(3)];
    
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

    % Obteción de las medidas del laser
    laser = apoloGetLaserLandMarks('LMS100');
    Zk_=[]; 
    Zk = [];
    Hk=[];
    Vec=[];

    % Se calcula la estimación para las balizas que se ven
    if isempty(laser.id)
        Pk=P_k;
        Xk=X_k;
    else
        for i=1:length(laser.id)

            ID=laser.id(i);
            Zk_ = [Zk_ ;sqrt((balizas(laser.id(i),1)-Xk_1(1))^2+(balizas(laser.id(i),2)-Xk_1(2))^2);
                    atan2(balizas(laser.id(i),2)-Xk_1(2),balizas(laser.id(i),1)-Xk_1(1))-Xk_1(3)];

            Zk = [Zk;laser.distance(i);laser.angle(i)];

            % Jacobiana de la matriz de observación
            
            Hk=[Hk;-((balizas(laser.id(i),1))-X_k(1))/(sqrt((balizas(laser.id(i),1)-Xk_1(1))^2+ (balizas(laser.id(i),2)-Xk_1(2))^2)) -((balizas(laser.id(i),2))-X_k(2))/(sqrt((balizas(laser.id(i),1)-Xk_1(1))^2+ (balizas(laser.id(i),2)-Xk_1(2))^2)) 0;   
            ((balizas(laser.id(i),2)-X_k(2))/((balizas(laser.id(i),1)-X_k(1))^2+(balizas(laser.id(i),2)-X_k(2))^2)) (-(balizas(laser.id(i),1)-X_k(1))/((balizas(laser.id(i),1)-X_k(1))^2+(balizas(laser.id(i),2)-X_k(2))^2)) -1];
        end


        % Comparacion
        Yk = Zk-Zk_;
        a=size(Yk)/2;

        if(a==0)
            Xk = X_k;
            Pk = P_k;
        else
            for r=1:a(1,1)
                if Yk(r*2)>pi
                    Yk(r*2) = Yk(r*2) - 2*pi;
                end
                if Yk(r*2)<(-pi)
                    Yk(r*2) = Yk(r*2) + 2*pi;
                end
            end
            for h=1:a
                Vec=[Vec vdist vang];
            end
            Rk = diag(Vec);
            Sk = Hk*P_k*((Hk)') + Rk;
            Wk = P_k*((Hk)')*inv(Sk);
            Xk = X_k + Wk*Yk;
            Pk = P_k-Wk*Hk*P_k;
        end
    end
    P_valores=[Pk(1,1),Pk(2,2),Pk(3,3)];
    e=[Xk(1)-pos_real_i(1);Xk(2)-pos_real_i(2);Xk(3)-pos_real_i(4)]
    e_acumulado=[e_acumulado;e'];


    %% Almacenar variables
    [time,pos_real,odometry_mes,odometry_cor,P_acumulado,C_acumulado] = acumular(time,pos_real,odometry_mes,odometry_cor,P_acumulado, C_acumulado,pos_real_i,odometry_mes_i,Xk',P_valores,correccion');

    %% Actualizar bucle
    apoloUpdate();
    apoloResetOdometry(robot,[Xk(1) Xk(2) Xk(3)]);
    pause(time_step/2);
    setTime(time_step);
    if time_i > simulation_time
        break;
    end
end

%plot
figure()
plot(pos_real(:,1),pos_real(:,2)); 
hold on;
grid on;
plot(odometry_cor(:,1),odometry_cor(:,2));
plot(odometry_mes(:,1),odometry_mes(:,2));
xlabel('time (s)');
ylabel('x (m)');
title('x');
legend('real','ekf','odometry')

figure()
plot(time,P_acumulado(:,1)); 
hold on;
grid on;
plot(time,P_acumulado(:,2)); 
plot(time,P_acumulado(:,3)); 
legend('Px','Py','Ptheta')

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