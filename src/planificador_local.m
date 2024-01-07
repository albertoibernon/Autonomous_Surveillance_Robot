function [vel_comando,pthObj] = planificador_local(planner,start,goal,time_step)
%%% PLANIFICADOR LOCAL %%%
%%% Función para generar puntos intermedios entre la posición estimada y el 
%   siguiente waypoint. La generación de puntos están basados en splines
%   que cumplen con las restricciones holonómicas del robot

    %% Optimización adicional una vez alcanzado el objetivo, aumentando el coste computacional
    % planner.ContinueAfterGoalReached = true;
    
    % Distancia máxima entre dos estados [m]
    planner.MaxConnectionDistance = 1.5;
    
    % Número máximo de iteraciones
    planner.MaxIterations = 1000;
    
    % Asignar una función que determine la finalización del algoritmo RRT*
    % basado en la distancia del punto actual al objetivo
    planner.GoalReachedFcn = @exampleHelperCheckIfGoal;
    
    % Generación de números aletarios. Semilla = 1 para repeteción de secuecia
    rng(1,'twister')
    
    % Planear una trayectoria
    [pthObj, solnInfo] = plan(planner,start,goal);

    % Definición de la interpolación (definir número de puntos necesarios
    % para mantener una velocidad de avance de 0.5 m/s en cada time step)
    diff_pose = norm(diff(pthObj.States(1:2,1:2)));
    interpolate(pthObj,2*round(length(pthObj.States(:,1))/(time_step/diff_pose)))
    
    % Plotear planificación local: puntos intermedios generados
    figure(1);
    plot(pthObj.States(:,1), pthObj.States(:,2), '.k',"MarkerSize",3)
    
    %% Comandos de velocidad de avance y velocidad de rotación (Control Activo)
    % Criterio: velocidad de avance constante
    vel_avance = vecnorm(diff(pthObj.States(:,1:2)),2,2)./time_step;
    vel_rotacion = diff(pthObj.States(:,3))./time_step;
    vel_comando = [vel_avance vel_rotacion];

end