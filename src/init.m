function pos_real = init(robot,pose_init)

global time_i

% Tiempo inicial
time_i = 0;

% Posición y rotación inicial del robot
init_position = [pose_init(1) pose_init(2) 0]; % [m]
init_rotation = pose_init(3); % [rad]

% Para la calibración
% init_position = [0 0 0];

apoloPlaceMRobot(robot,init_position,init_rotation);
apoloUpdate();
 
% Reseteo inicial de la navegación
pos_real = apoloGetLocationMRobot(robot);
apoloResetOdometry(robot,[pos_real(1:2) pos_real(4)]);

end