function [mytoc, len, smooth] = planificador_global(mTR, VD, mCD)
%%% PLANIFICADOR GLOBAL %%%
%%% Función para localizar waypoints desde el punto de inicio al punto
%   objetivo a partir del algoritmo de planificación RRT* cumpliendo con
%   las restricciones de obstáculos del mapa y con las restricciones
%   holonómicas del robot
    
    start = [1, 1, 0];
    goal = [2 , 27, 0];
    %% Configuración del mapa
    path_map_fig = './map_size_12.png'; % Nombre del mapa para cargar
    threshold    = 0.1; % Umbral para deterinar qué píxeles están libres u ocupados
    scale        = 10; % Escala del mapa
    side         = 40; % Dimensiones del mapa
    
    %% Cargar imagen del mapa, procesado del mapa y transformación en mapa de ocupación binaria 
    map = generateMap(path_map_fig, threshold, scale, side);
    
    %% Plotear mapa, punto inicial, punto objetivo, dirección inicial y dirección de llegada
    show(map)
    hold on
    width = 2.5;
    plot(start(1), start(2), 'ro', LineWidth=width)
    plot(goal(1), goal(2), 'mo', LineWidth=width)
    r = 1; width = 2;
    plot([start(1), start(1) + r*cos(start(3))], [start(2), start(2) + r*sin(start(3))], 'r-', LineWidth=width)
    plot([goal(1), goal(1) + r*cos(goal(3))], [goal(2), goal(2) + r*sin(goal(3))], 'm-', LineWidth=width)
    hold off
    
    %% Definición de los límites del mapa
    bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    
    %% Definición de Dubins State Space
    ss = stateSpaceDubins(bounds);
    ss.MinTurningRadius = mTR;
    
    %% Definición del validador espacio de estados 
    stateValidator = validatorOccupancyMap(ss); 
    stateValidator.Map = map;
    stateValidator.ValidationDistance = VD; % El planificador verifica si 
    % hay algún obstáculo a esta distancia a lo largo de la trayectoria
    
    %% Crear la planificación de trayectorias basada en RRT*
    planner = plannerRRTStar(ss, stateValidator);
    
    %% Optimización adicional una vez alcanzado el objetivo, aumentando el coste computacional
    % planner.ContinueAfterGoalReached = true;
    
    % Distancia máxima entre dos estados [m]
    planner.MaxConnectionDistance = mCD;
    
    % Número máximo de iteraciones
    planner.MaxIterations = 60000;
    
    % Asignar una función que determine la finalización del algoritmo RRT*
    % basado en la distancia del punto actual al objetivo
    planner.GoalReachedFcn = @exampleHelperCheckIfGoal;
    
    % Generación de números aletarios. Semilla = 1 para repeteción de secuecia
    rng(0,'twister')
    
    %% Plan a path.
    mytic = tic;
    [pthObj, solnInfo] = plan(planner, start, goal);
    mytoc = toc(mytic);
    disp("mTR = " + num2str(mTR));
    disp("VD = " + num2str(VD));
    disp("mCD = " + num2str(mCD));
    disp("Time = " + num2str(mytoc));
    len = pathLength(pthObj);
    disp("Path length = " + num2str(len));
    pathMetricsObj = pathmetrics(pthObj,stateValidator);
    smooth = smoothness(pathMetricsObj);
    disp("Path smoothness = " + num2str(smooth));
    
    %% Mostrar waypoints y grafo generado
    show(map)
    hold on
    alpha = 0.1; % Transparencia del grafo
    plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2),'.-','Color',[0 0 1 alpha]);
    plot(pthObj.States(:,1),pthObj.States(:,2),"*r","MarkerSize",5);

end
