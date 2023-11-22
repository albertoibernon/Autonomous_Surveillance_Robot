function matches=matching(mapa,dist_real,pos)
    
    % distancia real -> distancia obtenida tras kmeans, 
    % clusterización de los inliers y obtención del mínimo de cada cluster

    % distancia estimada -> distancia del mapa  
    % Inicializar matrices para almacenar las diferencias

    % margen para considerar obstaculo o mala medición
    margen=0.2;

    dist_est=[];
    for j = 1:length(mapa)
        p1=[mapa(j,1) mapa(j,2)];
        p2=[mapa(j,3) mapa(j,4)];
        dist=get_distance(pos,p1,p2);
        dist_est=[dist_est,dist(1)];
    end
    diffDistancias = zeros(length(dist_real), length(dist_est));

    % Calcular las diferencias entre distancias y ángulos
    for i = 1:length(dist_real)
        for j = 1:length(dist_est)
            diffDistancias(i, j) = abs(dist_real(i) - dist_est(j));
        end
    end
    b=size(diffDistancias);
    matches=[];
    for i=1:b(1)
        fila=diffDistancias(i,:);
        match=min(fila);
        if(match < margen)
            matches=[matches,match];
    end
end
