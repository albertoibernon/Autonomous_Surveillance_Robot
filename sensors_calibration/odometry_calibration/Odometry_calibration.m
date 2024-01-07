close all; clear; clc;

% Configuración


% which_plot = 1; % 1=x, 2=y, 3=xy, 4=\theta;

for which_plot = [1 2 3 4];

    robot = 'Marvin';
    laser = 'LMS100';
    time_step       = 0.1;
    simulation_time = 50;
    
    % Inicialización
    time_i = 0;
    
    % Posición y rotación inicial del robot
    if which_plot == 1
        init_position = [0 0 0]; % [m]
        init_rotation =  0; % [rad]
    elseif which_plot == 2
        init_position = [0 0 0]; % [m]
        init_rotation =  pi/2; % [rad] \pi/2
    elseif which_plot == 3
        init_position = [0 0 0]; % [m]
        init_rotation =  pi/4; % [rad]
    else
        init_position = [25 25 0]; % [m]
        init_rotation =  0; % [rad]
    end
    
    apoloPlaceMRobot(robot,init_position,init_rotation);
    apoloUpdate();
     
    % Reseteo inicial de la navegación
    pos_real_init = apoloGetLocationMRobot(robot);
    apoloResetOdometry(robot,[pos_real_init(1:2) pos_real_init(4)]);
    
    time = []; pos_real = []; odometry_mes = []; lidar_mes = [];
    
    % Bucle de Simulación
    while true
        %% Planificador
        if which_plot == 4
            speeds = [0 1];
        else
            speeds = [1 0];
        end

        %% DKE 
        apoloMoveMRobot(robot,speeds,time_step);
        apoloUpdate();
        % Obtener la posición real
        pos_real_i = apoloGetLocationMRobot(robot);
        %% Odometría (Sensores Propioceptivos)
        odometry_mes_i = apoloGetOdometry(robot); % [x(m), y(m), theta(rad)]
    
        %% Almacenar variables
        time         = [time; time_i];
        pos_real     = [pos_real; pos_real_i];
        odometry_mes = [odometry_mes; odometry_mes_i];
        
        
        %% Actualizar bucle
        apoloResetOdometry(robot,[pos_real_i(1:2) pos_real_i(4)]);
        pause(time_step/2)
        time_i = time_i + time_step;
        clc;
        fprintf('Time: %f',time_i)
        if time_i > simulation_time
            break;
        end
    end
    
    %% Save outputs
    if which_plot == 1
        save('output_x\time_calibration.mat','time')
        save('output_x\pos_real_calibration.mat','pos_real')
        save('output_x\odometry_calibration.mat','odometry_mes')
    
    elseif which_plot == 2
        save('output_y\time_calibration.mat','time')
        save('output_y\pos_real_calibration.mat','pos_real')
        save('output_y\odometry_calibration.mat','odometry_mes')
    
    elseif which_plot == 3
        save('output_xy\time_calibration.mat','time')
        save('output_xy\pos_real_calibration.mat','pos_real')
        save('output_xy\odometry_calibration.mat','odometry_mes')

    else 
        save('output_theta\time_calibration.mat','time')
        save('output_theta\pos_real_calibration.mat','pos_real')
        save('output_theta\odometry_calibration.mat','odometry_mes')
    
    end
    
    clearvars -except which_plot; close all; clc;
    
    % which_plot = 1;
    
    if which_plot==1
        path = "C:\Users\Josep\Desktop\Autonomous_Surveillance_Robot_Iss1\output_x";
        load('output_x\time_calibration.mat','time')
        load('output_x\pos_real_calibration.mat','pos_real')
        load('output_x\odometry_calibration.mat','odometry_mes')
        
        theta_real_diff = [0; diff(pos_real(:,4))];
        theta_real_diff(abs(theta_real_diff)>1) = theta_real_diff(abs(theta_real_diff)>1) + 2*pi;
        theta_meas_diff = [0; diff(odometry_mes(:,3))];
        theta_meas_diff(abs(theta_meas_diff)>1) = theta_meas_diff(abs(theta_meas_diff)>1) + 2*pi;
        
        error_x     = odometry_mes(:,1) - pos_real(:,1);
        error_y     = odometry_mes(:,2) - pos_real(:,2);
        error_theta = (theta_meas_diff-theta_real_diff)*180/pi;
        
        %% Post-pro
        figure();plot(time,odometry_mes(:,1),'r','LineWidth',1.2); hold on;grid on;plot(time,pos_real(:,1),'b--','LineWidth',1.2);xlabel('time (s)');ylabel('x (m)');title('x');legend('measured','real');
        saveas(gcf, fullfile(path, 'x_vs_time'), 'png');
        figure();plot(time,odometry_mes(:,2),'r','LineWidth',1.2); hold on;grid on;plot(time,pos_real(:,2),'b--','LineWidth',1.2);xlabel('time (s)');ylabel('y (m)');title('y');legend('measured','real');
        saveas(gcf, fullfile(path, 'y_vs_time'), 'png');
        figure();plot(odometry_mes(:,1),odometry_mes(:,2),'r','LineWidth',1.2);hold on;grid on;plot(pos_real(:,1),pos_real(:,2),'b--','LineWidth',1.2);xlabel('x (m)');ylabel('y (m)');title('trajectory');legend('measured','real');
        saveas(gcf, fullfile(path, 'x_vs_y'), 'png');
        figure();plot(time,theta_meas_diff,'r','LineWidth',1.2); hold on;grid on;plot(time,theta_real_diff.*180/pi,'b--','LineWidth',1.2);xlabel('time (s)');ylabel('\theta (deg)');title('\theta');legend('measured','real');
        saveas(gcf, fullfile(path, 'theta_vs_t'), 'png');
        figure();plot(time,error_x,'r','LineWidth',1.2);hold on; grid on;plot(time,error_y,'b--','LineWidth',1.2);legend('x error','y error');xlabel('time (s)');ylabel('Position Error (m)');title('Position Error');
        saveas(gcf, fullfile(path, 'position_error'), 'png');
        figure();plot(time,error_theta,'r','LineWidth',1.2);hold on; grid on;xlabel('time (s)');ylabel('\theta Error (deg)');title('Rotation Error');
        saveas(gcf, fullfile(path, 'theta_error'), 'png');
    
        % Varianza
        figure();histogram(error_x,50);xlabel('x Position (m)');ylabel('Number of Cases (-)');title('Advance Error');
        saveas(gcf, fullfile(path, 'varianza_x'), 'png');
        figure();histogram(error_theta,50);xlabel('\theta (deg)');ylabel('Number of Cases (-)');title('Rotation Error');
        saveas(gcf, fullfile(path, 'varianza_theta'), 'png');
    
        avance_covarianza = std(error_x)^2;
        rotacion_covarianza = std(error_theta)^2;
    
        save("output_x\avance_covarianza.mat","avance_covarianza");
        save("output_x\rotacion_covarianza.mat","rotacion_covarianza");
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif which_plot == 2
        path = "C:\Users\Josep\Desktop\Autonomous_Surveillance_Robot_Iss1\output_y";
        load('output_y\time_calibration.mat','time')
        load('output_y\pos_real_calibration.mat','pos_real')
        load('output_y\odometry_calibration.mat','odometry_mes')
        
        theta_real_diff = [0; diff(pos_real(:,4))];
        theta_real_diff(abs(theta_real_diff)>1) = theta_real_diff(abs(theta_real_diff)>1) + 2*pi;
        theta_meas_diff = [0; diff(odometry_mes(:,3))];
        theta_meas_diff(abs(theta_meas_diff)>1) = theta_meas_diff(abs(theta_meas_diff)>1) + 2*pi;
        
        error_x     = odometry_mes(:,1) - pos_real(:,1);
        error_y     = odometry_mes(:,2) - pos_real(:,2);
        error_theta = (theta_meas_diff-theta_real_diff)*180/pi;
        
        %% Post-pro
        figure();plot(time,odometry_mes(:,1),'r','LineWidth',1.2); hold on;grid on;plot(time,pos_real(:,1),'b--','LineWidth',1.2);xlabel('time (s)');ylabel('x (m)');title('x');legend('measured','real');
        saveas(gcf, fullfile(path, 'x_vs_time'), 'png');
        figure();plot(time,odometry_mes(:,2),'r','LineWidth',1.2); hold on;grid on;plot(time,pos_real(:,2),'b--','LineWidth',1.2);xlabel('time (s)');ylabel('y (m)');title('y');legend('measured','real');
        saveas(gcf, fullfile(path, 'y_vs_time'), 'png');
        figure();plot(odometry_mes(:,1),odometry_mes(:,2),'r','LineWidth',1.2);hold on;grid on;plot(pos_real(:,1),pos_real(:,2),'b--','LineWidth',1.2);xlabel('x (m)');ylabel('y (m)');title('trajectory');legend('measured','real');
        saveas(gcf, fullfile(path, 'x_vs_y'), 'png');
        figure();plot(time,theta_meas_diff,'r','LineWidth',1.2); hold on;grid on;plot(time,theta_real_diff.*180/pi,'b--','LineWidth',1.2);xlabel('time (s)');ylabel('\theta (deg)');title('\theta');legend('measured','real');
        saveas(gcf, fullfile(path, 'theta_vs_t'), 'png');
        figure();plot(time,error_x,'r','LineWidth',1.2);hold on; grid on;plot(time,error_y,'b--','LineWidth',1.2);legend('x error','y error');xlabel('time (s)');ylabel('Position Error (m)');title('Position Error');
        saveas(gcf, fullfile(path, 'position_error'), 'png');
        figure();plot(time,error_theta,'r','LineWidth',1.2);hold on; grid on;xlabel('time (s)');ylabel('\theta Error (deg)');title('Rotation Error');
        saveas(gcf, fullfile(path, 'theta_error'), 'png');
    
    
        % Varianza
        figure();histogram(error_x,50);xlabel('x Position (m)');ylabel('Number of Cases (-)');title('Advance Error');
        saveas(gcf, fullfile(path, 'varianza_x'), 'png');
        figure();histogram(error_y,50);xlabel('y Position (m)');ylabel('Number of Cases (-)');title('Advance Error');
        saveas(gcf, fullfile(path, 'varianza_y'), 'png');
        figure();histogram(error_theta,50);xlabel('\theta (deg)');ylabel('Number of Cases (-)');title('Rotation Error');
        saveas(gcf, fullfile(path, 'varianza_theta'), 'png');
    
        avance_covarianza_x = std(error_x)^2;
        avance_covarianza_y = std(error_y)^2;
        rotacion_covarianza = std(error_theta)^2;
    
        save("output_y\avance_covarianza_x.mat","avance_covarianza_x");
        save("output_y\avance_covarianza_y.mat","avance_covarianza_y");
        save("output_y\rotacion_covarianza.mat","rotacion_covarianza");
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif which_plot == 3
        path = "C:\Users\Josep\Desktop\Autonomous_Surveillance_Robot_Iss1\output_xy";
        load('output_xy\time_calibration.mat','time')
        load('output_xy\pos_real_calibration.mat','pos_real')
        load('output_xy\odometry_calibration.mat','odometry_mes')
        
        theta_real_diff = [0; diff(pos_real(:,4))];
        theta_real_diff(abs(theta_real_diff)>1) = theta_real_diff(abs(theta_real_diff)>1) + 2*pi;
        theta_meas_diff = [0; diff(odometry_mes(:,3))];
        theta_meas_diff(abs(theta_meas_diff)>1) = theta_meas_diff(abs(theta_meas_diff)>1) + 2*pi;
        
        error_x     = odometry_mes(:,1) - pos_real(:,1);
        error_y     = odometry_mes(:,2) - pos_real(:,2);
        error_theta = (theta_meas_diff-theta_real_diff)*180/pi;
        
        %% Post-pro
        figure();plot(time,odometry_mes(:,1),'r','LineWidth',1.2); hold on;grid on;plot(time,pos_real(:,1),'b--','LineWidth',1.2);xlabel('time (s)');ylabel('x (m)');title('x');legend('measured','real');
        saveas(gcf, fullfile(path, 'x_vs_time'), 'png');
        figure();plot(time,odometry_mes(:,2),'r','LineWidth',1.2); hold on;grid on;plot(time,pos_real(:,2),'b--','LineWidth',1.2);xlabel('time (s)');ylabel('y (m)');title('y');legend('measured','real');
        saveas(gcf, fullfile(path, 'y_vs_time'), 'png');
        figure();plot(odometry_mes(:,1),odometry_mes(:,2),'r','LineWidth',1.2);hold on;grid on;plot(pos_real(:,1),pos_real(:,2),'b--','LineWidth',1.2);xlabel('x (m)');ylabel('y (m)');title('trajectory');legend('measured','real');
        saveas(gcf, fullfile(path, 'x_vs_y'), 'png');
        figure();plot(time,theta_meas_diff,'r','LineWidth',1.2); hold on;grid on;plot(time,theta_real_diff.*180/pi,'b--','LineWidth',1.2);xlabel('time (s)');ylabel('\theta (deg)');title('\theta');legend('measured','real');
        saveas(gcf, fullfile(path, 'theta_vs_t'), 'png');
        figure();plot(time,error_x,'r','LineWidth',1.2);hold on; grid on;plot(time,error_y,'b--','LineWidth',1.2);legend('x error','y error');xlabel('time (s)');ylabel('Position Error (m)');title('Position Error');
        saveas(gcf, fullfile(path, 'position_error'), 'png');
        figure();plot(time,error_theta,'r','LineWidth',1.2);hold on; grid on;xlabel('time (s)');ylabel('\theta Error (deg)');title('Rotation Error');
        saveas(gcf, fullfile(path, 'theta_error'), 'png');
    
        % Varianza
        figure();histogram(error_x,50);xlabel('x Position (m)');ylabel('Number of Cases (-)');title('Advance Error');
        saveas(gcf, fullfile(path, 'varianza_x'), 'png');
        figure();histogram(error_y,50);xlabel('y Position (m)');ylabel('Number of Cases (-)');title('Advance Error');
        saveas(gcf, fullfile(path, 'varianza_y'), 'png');
        figure();histogram(error_theta,30);xlabel('\theta (deg)');ylabel('Number of Cases (-)');title('Rotation Error');
        saveas(gcf, fullfile(path, 'varianza_theta'), 'png');
        
        avance_covarianza_x = std(error_x)^2;
        avance_covarianza_y = std(error_y)^2;
        rotacion_covarianza = std(error_theta)^2;
        save("output_xy\avance_covarianza_x.mat","avance_covarianza_x");
        save("output_xy\avance_covarianza_x.mat","avance_covarianza_y");
        save("output_xy\rotacion_covarianza.mat","rotacion_covarianza");
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    else
        path = "C:\Users\Josep\Desktop\Autonomous_Surveillance_Robot_Iss1\output_theta";
        load('output_theta\time_calibration.mat','time')
        load('output_theta\pos_real_calibration.mat','pos_real')
        load('output_theta\odometry_calibration.mat','odometry_mes')
        
        theta_real_diff = [0; diff(pos_real(:,4))];
        theta_real_diff(abs(theta_real_diff)>1) = theta_real_diff(abs(theta_real_diff)>1) + 2*pi;
        theta_meas_diff = [0; diff(odometry_mes(:,3))];
        theta_meas_diff(abs(theta_meas_diff)>1) = theta_meas_diff(abs(theta_meas_diff)>1) + 2*pi;
        
        error_x     = odometry_mes(:,1) - pos_real(:,1);
        error_y     = odometry_mes(:,2) - pos_real(:,2);
        error_theta = (theta_meas_diff-theta_real_diff)*180/pi;
        
        %% Post-pro
        figure();plot(time,theta_meas_diff.*180/pi,'r','LineWidth',1.2); hold on;grid on;plot(time,theta_real_diff.*180/pi,'b--','LineWidth',1.2);xlabel('time (s)');ylabel('\theta (deg)');title('\theta');legend('measured','real')
        saveas(gcf, fullfile(path, 'theta_vs_t'), 'png');
        figure();plot(time,error_theta,'r','LineWidth',1.2);hold on; grid on;xlabel('time (s)');ylabel('\theta Error (deg)');title('Rotation Error');
        saveas(gcf, fullfile(path, 'theta_error'), 'png');
        figure();plot(time,odometry_mes(:,3).*180/pi,'r','LineWidth',1.2); hold on;grid on;plot(time,pos_real(:,4).*180/pi,'b--','LineWidth',1.2);xlabel('time (s)');ylabel('theta (deg)');title('theta');legend('measured','real')
        saveas(gcf, fullfile(path, 'theta'), 'png');
        % Varianza
        figure();histogram(error_theta,50);xlabel('\theta (deg)');ylabel('Number of Cases (-)');title('Rotation Error');
        saveas(gcf, fullfile(path, 'varianza_theta'), 'png');
    
        rotacion_covarianza = std(error_theta)^2;
        save("output_theta\avance_covarianza.mat","rotacion_covarianza");
    end
end
