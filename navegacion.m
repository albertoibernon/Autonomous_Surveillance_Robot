function [dist_real,ang_real,cuadrantes,puntos_inter,rectas]=navegacion(points)

%maxClusters=10; %Máxima cantidad de paredes que se prevee detectar a la vez
Radio=19.9; %20 metros 

% Parámetros de RANSAC
numIteraciones = 500;  % Número de iteraciones de RANSAC
sampleSize = 2; % number of points to sample per trial
maxDistance=0.01;
min_num_points=10;

points_aux=points;
counter=0;
for i = 1:length(points_aux)
    value1 = abs(points_aux(i,1));
    value2 = abs(points_aux(i,2));
    value=sqrt(value1^2+value2^2);
    if value>Radio
        points(i-counter,:) = [];
        counter=counter+1;
    end
end

%Obtengo los clusters
%idx = clusterdata(points,'Criterion','distance','Cutoff',0.1,'Distance','mahalanobis');
%idx = clusterdata(points,'Criterion','distance','Cutoff',0.5,'Distance','chebychev');
idx = clusterdata(points,'Criterion','distance','Cutoff',0.2,'Distance','squaredeuclidean');
index=1;
numClusters = max(idx);
clusters={numClusters};
for j=1:numClusters
    counter=1;
    cluster=zeros(0,0);
    for i=1:length(idx)
        value = idx(i,1);
        if value == index
            cluster(counter,1)=points(i,1);
            cluster(counter,2)=points(i,2);
            counter=counter+1;
        end
    end
    clusters{index}=cluster;
    index=index+1;
end

% Filtrado
for i=1:length(clusters)
    clus=clusters{i};
    if(length(clus)<min_num_points)
        clusters{i}=[];
    end
end

% Crear un índice lógico para los elementos a mantener


% Plotear los puntos con colores correspondientes a los clusters
% figure;
% gscatter(points(:,1), points(:,2), idx);
% for i=1:length(clusters)
%     clus=clusters{i};
%     tam=size(clus);
%     if(tam(1)>min_num_points)
%         plot(clus(:,1), clus(:,2),'o');
%         hold on
%     end
% end

dist_real=[];
ang_real=[];
cuadrantes=[];
puntos_inter=[];
rectas=[];
num_recta=0;
for j=1:length(clusters)
    clus=clusters{j};
    tam=size(clus);
    if(tam(1)>min_num_points)
        % Ransac de cada cluster
        fitLineFcn = @(clus) polyfit(clus(:,1),clus(:,2),1); % fit function using polyfit
        evalLineFcn = @(model, clus) sum((clus(:, 2) - polyval(model, clus(:,1))).^2,2);
        while true
            puntos=size(clus);
            if puntos(1)>min_num_points
                [~, inlierIdx] = ransac(clus,fitLineFcn,evalLineFcn, sampleSize,maxDistance,"MaxNumTrials",numIteraciones);
                modelInliers = polyfit(clus(inlierIdx,1),clus(inlierIdx,2),1);
                inlierPts = clus(inlierIdx,:);
                p1 = cat(2,inlierPts(1,1),inlierPts(1,2));
                p2 = cat(2,inlierPts(end,1),inlierPts(end,2));
                x = [min(inlierPts(:,1)) max(inlierPts(:,1))];
                y = modelInliers(1)*x + modelInliers(2); %definción de la recta
                value = get_distance([0 0], p1, p2);
                dist_real=[dist_real,value(1)];
                ang_real=[ang_real,value(4)];
                cuadrantes = [cuadrantes,value(5)];
                puntos_inter = [puntos_inter; [value(2),value(3)]];
                rectas = [rectas; [value(6),value(7),value(8)]];
                num_recta=num_recta+1;
                % plot(x, y);
                % hold on
                % plot([0 value(2)],[0 value(3)]);
                % text([value(2)],[value(3)],[num2str(num_recta)]);
                counter=0;
                for i=1:length(inlierIdx)
                    value = inlierIdx(i,1);
                    if value == 1
                        clus(i-counter,:) = [];
                        counter=counter+1;
                    end
                end
            else
                break
            end
        end
    end
end
% clf
end