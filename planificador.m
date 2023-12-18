function speeds = planificador
    global time_i
    
    %% Close-loop
    vel_avance  = 1; % Avance [m/s]
    vel_rot_max = 0.5; % Avance [m/s]

    %% Open loop
    if time_i < 8.5
        speeds = [1 0]; % Avance y rotación [m/s rad/s]

    elseif time_i >= 13.7 && time_i < 15.3
        speeds = [1 1]; % Avance y rotación [m/s rad/s]

    elseif time_i >= 27.7 && time_i < 29.3
        speeds = [1 -1]; % Avance y rotación [m/s rad/s]

    elseif time_i >= 36 && time_i < 37.6
        speeds = [1 -1]; % Avance y rotación [m/s rad/s]

    elseif time_i >= 45 && time_i < 46
        speeds = [1 -1]; % Avance y rotación [m/s rad/s]

        elseif time_i >= 55 && time_i < 55.5
        speeds = [1 -1]; % Avance y rotación [m/s rad/s]

        elseif time_i >= 68 && time_i < 69.6
        speeds = [1 -1]; % Avance y rotación [m/s rad/s]
    else
        speeds = [1 0]; % Avance y rotación [m/s rad/s]

    end

    %% Para la calibración
     %speeds = [0 0];
     speeds = [1 0]; % Avance y rotación [m/s rad/s]
end