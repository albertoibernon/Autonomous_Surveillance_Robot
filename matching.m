function [matches,H]=matching(mapa,dist_real,ang_real,pos,X)
    
    % distancia real -> distancia obtenida tras kmeans, 
    % clusterización de los inliers y obtención del mínimo de cada cluster

    % distancia estimada -> distancia del mapa  
    % Inicializar matrices para almacenar las diferencias

    % margen para considerar obstaculo o mala medición
    margen_distancia = 0.01;
    margen_angulo = 0.01;

    dist_est=zeros(1,31);
    ang_est=zeros(1,31);
    points_expected=[];
    for j = 1:length(mapa)
        p1=[mapa(j,1) mapa(j,2)];
        p2=[mapa(j,3) mapa(j,4)];
        dist=get_distance(pos,p1,p2);
        dist_est(j)=dist(1);
        ang_est(j)=dist(4);
        points_expected=cat(1,points_expected,[dist(2),dist(3)]);
    end
    diffDistancias = zeros(length(dist_real), length(dist_est));
    diffAngulos = zeros(length(dist_real), length(dist_est));
    % dist_real
    % dist_est
    % ang_est
    % ang_real

    % Calcular las diferencias entre distancias y ángulos
    for i = 1:length(dist_real)
        for j = 1:length(dist_est)
            diffAngulos(i, j) =  abs(ang_real(i) - ang_est(j));
            diffDistancias(i, j) = abs(dist_real(i) - dist_est(j));
        end
    end
    b=size(diffDistancias);
    matches=[];
    numeracion=[];
    errores=[];
    todosmatches=[];
    todosangulos=[];

    %% Calculo de la distancia de Mahalonobis
    % Y son las observaciones
    % X muestra de referencia 
    H=[];
    for i=1:b(1)
        fila=diffDistancias(i,:);
        [match,I]=min(fila);
        error_angulo=diffAngulos(i,I);
        todosmatches=[todosmatches,match];
        todosangulos=[todosangulos,error_angulo];
        dist_maha= mahal(match,X);
        % if dist_maha==Inf
        %     if(error_angulo < margen_angulo && match < margen_distancia )
        %     matches=[matches,match];
        %     numeracion=[numeracion,i];
        %     % points_expected(I,2)
        %     % dist_est(I)
        %     errores=[errores,error_angulo];
        %     Hx=-(points_expected(I,1)-pos(1))/(sqrt(dist_est(I)));
        %     Hy=-(points_expected(I,2)-pos(2))/(sqrt(dist_est(I)));
        %     H=cat(1,H,[Hx,Hy]);
        %     end
        % elseif(dist_maha<5)
        % 
        % matches=[matches,match];
        % numeracion=[numeracion,i];
        % % points_expected(I,2)
        % % dist_est(I)
        % errores=[errores,error_angulo];
        % Hx=-(points_expected(I,1)-pos(1))/(sqrt(dist_est(I)));
        % Hy=-(points_expected(I,2)-pos(2))/(sqrt(dist_est(I)));
        % H=cat(1,H,[Hx,Hy]);
        % 
        % end
        if(error_angulo < margen_angulo && match < margen_distancia )
            matches=[matches,match];
            numeracion=[numeracion,i];
            % points_expected(I,2)
            % dist_est(I)
            errores=[errores,error_angulo];
            Hx=-(points_expected(I,1)-pos(1))/(sqrt(dist_est(I)));
            Hy=-(points_expected(I,2)-pos(2))/(sqrt(dist_est(I)));
            H=cat(1,H,[Hx,Hy]);
        end
    end
    
matches = cat(1,matches,errores);
matches = cat(1,matches,numeracion);
%todosmatches=cat(1,todosmatches,todosangulos)
a=size(H);
fila3_H=zeros(1,a(1));
H=cat(2,H,fila3_H');
end
