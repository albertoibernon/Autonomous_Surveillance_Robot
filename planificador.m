function speeds = planificador(time_i)
    global time_i
    if time_i < 8.5
        speeds = [1 0]; % Avance y rotación [m/s rad/s]

    elseif time_i >= 13.7 && time_i < 15.3+1.15*4
        speeds = [0.25 0.25]; % Avance y rotación [m/s rad/s]

    elseif time_i >= 27.7+1.15*4 && time_i < 29.3+1.15*4*2
        speeds = [0.25 -0.25]; % Avance y rotación [m/s rad/s]

    elseif time_i >= 36+1.15*4*2 && time_i < 37.6+1.15*4*3
        speeds = [0.25 -0.25]; % Avance y rotación [m/s rad/s]

    elseif time_i >= 45+1.15*4*3 && time_i < 46+1.15*4*3+3
        speeds = [0.25 -0.25]; % Avance y rotación [m/s rad/s]

    elseif time_i >= 55+1.15*4*3+3 && time_i < 55.5+1.15*4*3+5
        speeds = [0.25 -0.25]; % Avance y rotación [m/s rad/s]
    else
        speeds = [1 0]; % Avance y rotación [m/s rad/s]

    end

    % Para la calibración
     %speeds = [0 0.1];
     %speeds = [1 -1]; % Avance y rotación [m/s rad/s]
end