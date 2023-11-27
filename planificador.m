function speeds = planificador(time_i)
    global time_i
    if time_i < 8.5
        speeds = [1 0]; % Avance y rotación [m/s rad/s]
    elseif time_i >= 13.7 && time_i < 15.3
        speeds = [1 1]; % Avance y rotación [m/s rad/s]
    else
        speeds = [1 0]; % Avance y rotación [m/s rad/s]
    end

    % Para la calibración
     %speeds = [0 0.1];
end