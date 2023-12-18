function [pthObj,vel_comando] = planificador_global(start,goal,time_step,simulation_time)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Path Planning Algorithm
%% Source: https://es.mathworks.com/matlabcentral/fileexchange/109485-lessons-on-mobile-robot-localization-and-kalman-filters
%% Date: 13/12/23
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Configuration Parameters
path_map_fig = './plano_1_cuadrado.jpg';
threshold = 0.1; % decide between free or obs
scale = 10*108/40; % resolution grid

%% Load an image, process it and convert it to binary occupancy map.
map = generateMap(path_map_fig, threshold, scale);

%% Show the start and goal positions of the robot
show(map)
hold on
width = 2.5;
plot(start(1), start(2), 'ro', LineWidth=width)
plot(goal(1), goal(2), 'mo', LineWidth=width)
% Show the start and goal headings
r = 1; width = 2;
plot([start(1), start(1) + r*cos(start(3))], [start(2), start(2) + r*sin(start(3))], 'r-', LineWidth=width)
plot([goal(1), goal(1) + r*cos(goal(3))], [goal(2), goal(2) + r*sin(goal(3))], 'm-', LineWidth=width)
hold off

%% Define map limits
bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

%% Define a state space as dubins.
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.1;

%% Define a state validator object by state space.
stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.1; % Planner check if there is an obstacle between this distance in the path.

%% Create RRT* path planner and set parameters.
planner = plannerRRTStar(ss, stateValidator);

%% The next parameter allows further optimization after goal is reached, however it increase computing time.
% planner.ContinueAfterGoalReached = true;

%% Define the maximum connection between two states in [m]
planner.MaxConnectionDistance = 1.5;

%% Define maximum iterations . (It depends on the size and shape of the map)
planner.MaxIterations = 35000;

%% Assign a function to know when to stop the RTT* algorithm based on distance from actual state to goal.
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

%% Define a control random number generator.The seed is 1 to get the "same result" each execution.
rng(1,'twister')

%% Plan a path.
tic
[pthObj, solnInfo] = plan(planner, start, goal);
toc

%% Show  the path calculated by planner and tree generated.
width = 2;
show(map)
hold on
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');
%% Interpolate and plot path
diff_pose = norm(diff(pthObj.States(1:2,1:2)));
factor = time_step/diff_pose;
interpolate(pthObj,round(length(pthObj.States(:,1))/factor))
plot(pthObj.States(:,1), pthObj.States(:,2), 'r*', 'LineWidth', 2)
%% Show the start and goal in the grid map
plot(start(1), start(2), 'ro', LineWidth=width)
plot(goal(1), goal(2), 'mo', LineWidth=width)
hold off

%% Vamos a calcular la posición y la velocidad de referencia en función de
% de un criterio (velocidad de avance constante) para dar las referencia en
% términos de velocidad (para así tener correlación con el tiempo) y saber
% cuándo aplicar cada comando. -> Control activo
% vel_avance = vecnorm(diff(pthObj.States(:,1:2)),2,2)./time_step;
vel_avance = ones(length(pthObj.States(:,1))-1,1);
vel_rotacion = diff(pthObj.States(:,3))./time_step;
vel_comando = [vel_avance vel_rotacion];

%% Además vamos a tener el control reactivo que corrige errores

end
