function [matches,H]=matching(mapa,dist_real,ang_real,pos,cuadrantes,puntos_inter,rectas)
    
    % distancia real -> distancia obtenida tras kmeans, 
    % clusterización de los inliers y obtención del mínimo de cada cluster

    % distancia estimada -> distancia del mapa  
    % Inicializar matrices para almacenar las diferencias

    % margen para considerar obstaculo o mala medición
    margen_distancia = 0.2;
    margen_angulo = 0.1;

    dist_est=zeros(1,31);
    ang_est=zeros(1,31);
    cuad_est=zeros(1,31);
    rectas_est=zeros(3,31);
    points_expected=[];
    for j = 1:length(mapa)
        p1=[mapa(j,1) mapa(j,2)];
        p2=[mapa(j,3) mapa(j,4)];
        dist=get_distance(pos,p1,p2);
        % hold on
        % plot([pos(1) dist(2)],[pos(2) dist(3)]);
        % text([pos(1) dist(2)],[pos(2) dist(3)],[num2str(j)]);
        dist_est(j)=dist(1);
        ang_est(j)=dist(4);
        cuad_est(j)=dist(5);
        points_expected=cat(1,points_expected,[dist(2),dist(3)]);
        rectas_est(:,j)=[dist(6);dist(7);dist(8)];
    end
    % clf
    diffDistancias = zeros(length(dist_real), length(dist_est))+99;
    diffAngulos = zeros(length(dist_real), length(dist_est))+99;
    % dist_real
    % dist_est
    % ang_est
    % ang_real

    % Calcular las diferencias entre distancias y ángulos
    for i = 1:length(dist_real)
        for j = 1:length(dist_est)
            if cuadrantes(i)-cuad_est(j)==0
                diffAngulos(i, j) =  abs(ang_real(i) - ang_est(j));
                diffDistancias(i, j) = abs(dist_real(i) - dist_est(j));
            end
        end
    end
    b=size(diffDistancias);
    matches=[];
    numeracion=[];
    errores=[];
    todosmatches=[];
    todosangulos=[];

    H=[];
    posiciones=[];
    for i=1:b(1)
        fila=diffDistancias(i,:);
        [match,I]=min(fila);
        error_angulo=diffAngulos(i,I);

        if(error_angulo < margen_angulo)
            %todosmatches=[todosmatches,match];
            %numeracion=[numeracion,j];
            %errores=[errores,error_angulo];

            % hold on 
            % plot([pos(1) points_expected(I,1)],[pos(2) points_expected(I,2)],'blue');
            % plot([pos(1) puntos_inter(i,1)+pos(1)],[pos(2) puntos_inter(i,2)+pos(2)],'red');
            % text([points_expected(I,1)],[points_expected(I,2)],[num2str(i)]);
            % text([puntos_inter(i,1)+pos(1)],[puntos_inter(i,2)+pos(2)],[num2str(i)]);
            A=rectas_est(1,I);
            B=rectas_est(2,I);
            C=rectas_est(3,I);
            A_med=rectas(i,1);
            B_med=rectas(i,2);
            C_med=rectas(i,3);

            % Dibujar la recta

            % if B==0
            % x_ = -C / A;
            % y_ = linspace(-1, 40, 100); 
            % plot([x_, x_], [min(y_), max(y_)], 'LineWidth', 2);  
            % 
            % x = -C_med / A_med + pos(1);
            % y = linspace(-1, 40, 100)+pos(2); 
            % plot([x, x], [min(y), max(y)],'r--', 'LineWidth', 2);  
            % else
            % x_ = linspace(0 - 1, 40, 100);
            % y_ = (-A*x_ - C) / B;
            % y = (-A_med*x_ - C_med) / B_med;
            % plot(x_, y_, 'b-', 'LineWidth', 2);
            % plot(x_+pos(1), y+pos(2), 'r--', 'LineWidth', 2);
            % end
            % matches=[matches,match];
            % Hx=-(points_expected(I,1)-pos(1))/(dist_est(I));
            % Hy=-(points_expected(I,2)-pos(2))/(dist_est(I));


            Hx = A/(sqrt(A^2+B^2));
            Hy = B/(sqrt(A^2+B^2));
            H=cat(1,H,[Hx,Hy]);
            matches=[matches,match];
        end
    end
    % clf
    
%matches = cat(1,matches,errores);
%matches = cat(1,matches,numeracion);
%todosmatches=cat(1,todosmatches,todosangulos);
a=size(H);
fila3_H=zeros(1,a(1));
H=cat(2,H,fila3_H');
end
