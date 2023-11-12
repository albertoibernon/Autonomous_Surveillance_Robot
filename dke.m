function pos_real = dke(robot,speeds,time_step)    
    % Comandar velocidad
    apoloMoveMRobot(robot,speeds,time_step);

    % Obtener la posición real
    pos_real = apoloGetLocationMRobot(robot);
end