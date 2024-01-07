function [comando_reactivo,up,ui,ud,error_pos,dist_sonic,ii] = control_reactivo(comando,pos_ref,pos_est_1,pos_est,vel_comando,obstaculo_flag,ii)
%% Función que proporciona el comando de velocidad de rotación para el control reactivo responsable de:
%  - Corregir la posición del robot con respecto a la posición de referencia
%  - Detectar y evitar obstáculos no pertenecientes al mapa
%  - Alejarse de las paredes en caso de aproximarse por debajo de una determinada distancia umbral

%  Inputs del controlador
%  - Posición de referencia
%  - Posición estimada en t
%  - Posición estimada en t-1
%  - Comando del control activo en t

    %% Controlador PI para corregir la posición w.r.t. la referencia
    % Ganancias del Controlador PID
    Kp = 0.6;    % Ganancia proporcional para la velocidad de rotación
    Ki = 0.0025; % Ganancia integral para la velocidad de rotación
    Kd = 0;      % Ganancia derivativa para la velocidad de rotación

    % Obtención de la distancia entre la posición de referencia y la posición estimada
    [error_pos,closest] = get_distance(pos_ref(1:2),pos_est_1(1:2),pos_est(1:2));
    A = pos_est_1(1:2) - closest;
    B = pos_ref(1:2) - closest;
    sign_ang = cross([A 0],[B 0]); % Obtención de la dirección lateral de la distancia
    error_pos = -1*sign(sign_ang(3))*error_pos; % Error de posición

    % Velocidad de rotación para la componente reactiva a partir del PID
    comando_reactivo = [0 0];
    [vel_rotacion,up,ui,ud] = pidController(error_pos, Kp, Ki, Kd);
    if any(vel_rotacion)
        comando_reactivo(2) = vel_rotacion;
    end
    
    %% Sensores ultrasónicos para detectar la distancia a las paredes y obstáculos
    dist_sonic = deteccion_colisiones(); % Distancia de los sensores de ultrasonidos

    %% Controlador P para alejarse de las paredes y de los obstáculos
    umbral_centro  = 1;   % Distancia mínima a las paredes frontales (m)
    umbral_lateral = 1;   % Distancia mínima a las paredes frontales (m)
    gain           = 0.4; % Ganancia para el control de detección de obstáculos

    % Detección de los obstáculos laterales
    if any([dist_sonic(1) dist_sonic(3)]<umbral_lateral)
        % Detección de los obstáculos frontales
        if dist_sonic(2)<umbral_centro % Obstáculo frontal detectado
            comando = [0 0];
            comando_reactivo(1) = 0.5; % Cuando haya un obstáculo la velocidad de avance se combina con una velocidad de giro
            closest_lateral = sign(dist_sonic(1) - dist_sonic(3));
            comando_reactivo(2) = closest_lateral*0.3; % Sentido de giro hacia el lado con mayor distancia a las paredes
            obstaculo_flag = 1; % Activación de la bandera de detección de obstáculos frontal
        
        elseif dist_sonic(2)>umbral_centro && obstaculo_flag == 1 % El robot se ha alejado del obstáculo
            ii = length(vel_comando(:,1)); % Regenerar una planificación local a partir de la nueva posición para esquivar el obstáculo
            obstaculo_flag = 0; % Desactivación de la bandera de detección de obstáculo frontal
        end
        
        % Corrección de la velocidad de rotación con un controlador en
        % función de la distancia a las paredes. Combina la distancia
        % lateral a ambos lados del robot. Da mayor peso a la distancia más
        % cercana para contrarrestarlo con un giro en sentido contrario.
        comando_reactivo(2) = comando_reactivo(2) + gain*(dist_sonic(1) - umbral_lateral) + gain*(umbral_lateral - dist_sonic(3));
    end

    % Control reactivo considerando el control activo (planificador local) 
    % más las contribuciones de los controladores de detección de obstáculos
    comando_reactivo = comando + comando_reactivo;

end