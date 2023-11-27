function output = get_distance(punto,p1,p2)

    distancia_limite1 = norm(punto - p1);
    distancia_limite2 = norm(punto - p2);
    limite_s_x = max(p1(1),p2(1));
    limite_s_y = max(p1(2),p2(2));
    limite_i_x = min(p1(1),p2(1));
    limite_i_y = min(p1(2),p2(2));

    % Calcular la distancia entre el robot y la recta del ransac 
    x1 = p1(1);
    y1 = p1(2);
    x2 = p2(1);
    y2 = p2(2);
    x0 = punto(1);
    y0 = punto(2);

    A = y2 - y1;
    B = x1 - x2;
    C = x2*y1 - x1*y2;
    distancia_ideal = abs(A*x0 + B*y0 + C) / sqrt(A^2 + B^2);
    final = [(B*(B*x0 - A*y0) - A*C)/(A^2 + B^2), (A*(A*y0 - B*x0) - B*C)/(A^2 + B^2)];

    if final(1)<limite_s_x  && final(1)>limite_i_x
    else
        if final(2)<limite_s_y  && final(2)>limite_i_y
        else
            if distancia_limite1>distancia_limite2
            final = p2;
            distancia_ideal = distancia_limite2;
            else
            final = p1;
            distancia_ideal = distancia_limite1;
            end
        end
    end

    % Parametros recta distancia
    A1 = final(2) - y0;
    B1 = x0 - final(1);

    dir_recta1 = [A, B];
    dir_recta2 = [A1, B1];

    % Obtenemos el ángulo que forman las dos rectas 
    cos_theta = dot(dir_recta1, dir_recta2) / (norm(dir_recta1) * norm(dir_recta2));
    angulo = acos(cos_theta);

    if abs(angulo) > 1.5708
        angulo = pi-abs(angulo);
    else
        angulo=abs(angulo);
    end

    output=[distancia_ideal,final,angulo];
end