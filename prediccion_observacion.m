function [dist_est,ang_est,points_est] = prediccion_observacion(mapa,pos_est)

% Predicción de la observación de la distanccia a las paredes del mapa  
dist_est   = zeros(1,31);
ang_est    = zeros(1,31);
points_est = [];
    for j = 1:length(mapa)
        p1                    = [mapa(j,1) mapa(j,2)];
        p2                    = [mapa(j,3) mapa(j,4)];
        [dist,closest,angulo] = get_distance(pos_est,p1,p2);
        dist_est(j)           = dist;
        ang_est(j)            = angulo;
        points_est            = [points_est; [closest(1) closest(2)]];
    end
end