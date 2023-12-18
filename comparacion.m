function [matches,H] = comparacion(dist_medida,pos_est,dist_est,points_est)

%%% Output:
% matches: error entre la distancia estimada y la distancia medida
% H: matriz de observación

% Distancia medida -> distancia obtenida tras kmeans, clusterización de 
% los inliers y obtención del mínimo de cada cluster

% Calcular las diferencias entre la predicción y la observación de la medida
diffDistancias = zeros(length(dist_medida), length(dist_est));
for i = 1:length(dist_medida)
    for j = 1:length(dist_est)
        diffDistancias(i,j) = abs(dist_medida(i) - dist_est(j));
    end
end

%% Matriz de Observaciones
% Y: Observaciones
% X: Vector de variables de estado ([x, y, theta])
H=[];
matches=[];
dim = size(diffDistancias);
for i=1:dim(1)
    fila=diffDistancias(i,:);
    [match,I]=min(fila);
    matches = [matches, match];
    Hx = -(points_est(I,1)-pos_est(1))/(dist_est(I));
    Hy = -(points_est(I,2)-pos_est(2))/(dist_est(I));
    H  = [H; [Hx,Hy]];
end
dim=size(H);
H= [H zeros(dim(1),1)];

end
