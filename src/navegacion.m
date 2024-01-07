function [Xk,Pk,Vec] = navegacion(Xk,Pk,Qk_1,balizas,odometry_mes_i,vdist,vang)
    % Nuevo ciclo (k-1 = k)
    Xk_1 = Xk;
    Pk_1 = Pk;
    Uk = [sqrt((Xk_1(1)-odometry_mes_i(1))^2+(Xk_1(2)-odometry_mes_i(2))^2);odometry_mes_i(3)-Xk_1(3)];

    % Prediccion del estado
    X_k = [(Xk_1(1) + Uk(1)*cos(Xk_1(3)+(Uk(2)/2)));
           (Xk_1(2) + Uk(1)*sin(Xk_1(3)+(Uk(2)/2)));
           (Xk_1(3) + Uk(2))];

    Ak = [1 0 (-Uk(1)*sin(Xk_1(3)+Uk(2)/2));
          0 1 (Uk(1)*cos(Xk_1(3)+Uk(2)/2));
          0 0 1                             ];
    Bk = [(cos(Xk_1(3)+Uk(2)/2)) (-0.5*Uk(1)*sin(Xk_1(3)+Uk(2)/2));
          (sin(Xk_1(3)+Uk(2)/2)) (0.5*Uk(1)*cos(Xk_1(3)+Uk(2)/2));
           0                     1                                 ];
    P_k = Ak*Pk_1*((Ak)') + Bk*Qk_1*((Bk)');

    % Obteción de las medidas de las balizas a partir del laser
    laser = apoloGetLaserLandMarks('LMS100');
    Zk_=[]; Zk = []; Hk=[]; Vec=[];

    % Se calcula la estimación para las balizas que se ven
    if isempty(laser.id)
        Pk=P_k;
        Xk=X_k;
    else
        for i=1:length(laser.id)
            % Predicción de la medida (distancia a las balizas)
            Zk_ = [Zk_ ;sqrt((balizas(laser.id(i),1)-Xk_1(1))^2+(balizas(laser.id(i),2)-Xk_1(2))^2);
                    atan2(balizas(laser.id(i),2)-Xk_1(2),balizas(laser.id(i),1)-Xk_1(1))-Xk_1(3)];

            Zk = [Zk;laser.distance(i);laser.angle(i)];

            % Jacobiana de la matriz de observación
            Hk=[Hk;-((balizas(laser.id(i),1))-X_k(1))/(sqrt((balizas(laser.id(i),1)-Xk_1(1))^2+ (balizas(laser.id(i),2)-Xk_1(2))^2)) -((balizas(laser.id(i),2))-X_k(2))/(sqrt((balizas(laser.id(i),1)-Xk_1(1))^2+ (balizas(laser.id(i),2)-Xk_1(2))^2)) 0;   
            ((balizas(laser.id(i),2)-X_k(2))/((balizas(laser.id(i),1)-X_k(1))^2+(balizas(laser.id(i),2)-X_k(2))^2)) (-(balizas(laser.id(i),1)-X_k(1))/((balizas(laser.id(i),1)-X_k(1))^2+(balizas(laser.id(i),2)-X_k(2))^2)) -1];
        end

        % Comparacion
        Yk = Zk-Zk_; % Innovación
        a=size(Yk)/2;
        if(a==0)
            Xk = X_k;
            Pk = P_k;
        else
            for r=1:a(1,1)
                if Yk(r*2)>pi
                    Yk(r*2) = Yk(r*2) - 2*pi;
                end
                if Yk(r*2)<(-pi)
                    Yk(r*2) = Yk(r*2) + 2*pi;
                end
            end
            for h=1:a
                Vec=[Vec vdist vang];
            end
            
            % Corrección
            Rk = diag(Vec);
            Sk = Hk*P_k*((Hk)') + Rk;
            Wk = P_k*((Hk)')*inv(Sk);
            Xk = X_k + Wk*Yk;
            Pk = P_k-Wk*Hk*P_k;
        end
    end

end