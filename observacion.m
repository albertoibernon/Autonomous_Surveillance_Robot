function dist_medida = observacion(points)

%%% Inputs:
% points: Nube de puntos generado por el lidar expresado en coordenadas Cartesianas

%%% Outputs:
% dist_medida: Mínima distancias (perpendiculares) entre la pose y las paredes del campo de visión
% ang_real:

% Parámetros de RANSAC
numIteraciones = 300;  % Número de iteraciones de RANSAC
sampleSize     = 3;    % Number of points to sample per trial
maxDistance    = 0.02; %0.001;
min_num_points = 5;

% Eliminar medidas que no detecten paredes al estar fuera del rango
radio_laser_rango = 19.9;
points_aux=points;
counter=0;
for i = 1:length(points_aux)
    if norm(points_aux(i,:)) > radio_laser_rango
        points(i-counter,:) = [];
        counter=counter+1;
    end
end

% Obtener los clusters (agrupación de la nube de puntos)
idx = clusterdata(points,'Criterion','distance','Cutoff',0.2,'Distance','squaredeuclidean');
index=1;
clusters_index = 1;
for j = 1:max(idx)
    if length(find(idx==index)) >= min_num_points
        clusters{clusters_index} = points(find(idx==index),:);
        clusters_index = clusters_index + 1;
    end
    index = index + 1;
end

% % Plotear los puntos con colores correspondientes a los clusters
% close; figure();hold on; grid minor;
% for i=1:length(clusters)
%     clus=clusters{i};
%     plot(clus(:,1), clus(:,2),'o');
% end

dist_medida=[];
for j=1:length(clusters)
    clus=clusters{j};
    % RANSAC de cada cluster
    fitLineFcn = @(clus) polyfit(clus(:,1),clus(:,2),1); % fit function using polyfit
    evalLineFcn = @(model, clus) sum((clus(:, 2) - polyval(model, clus(:,1))).^2,2);
    [~, inlierIdx] = ransac(clus,fitLineFcn,evalLineFcn, sampleSize,maxDistance,"MaxNumTrials",numIteraciones);
    inlierPts = clus(inlierIdx,:);
    p1 = inlierPts(1,:);
    p2 = inlierPts(end,:);

    [distancia,closest_point,angulo] = get_distance([0 0], p1, p2);

    dist_medida = [dist_medida, distancia];

    % plot([p1(1) p2(1)],[p1(2) p2(2)]);
    % plot([0 closest_point(1)],[0 closest_point(2)]);
    % text([closest_point(1)],[closest_point(2)],[num2str(j)]);
end

end