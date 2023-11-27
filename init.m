function init(robot)

global time_i

% Tiempo inicial
time_i = 0;

% Posición y rotación inicial del robot
init_position = [1 1 0]; % [m]
init_rotation = 0; % [rad]

% Para la calibración
% init_position = [19 3 0];

apoloPlaceMRobot(robot,init_position,init_rotation);
apoloUpdate();
 
% Reseteo inicial de la navegación
pos_real = apoloGetLocationMRobot(robot);
apoloResetOdometry(robot,pos_real(1:3));

end