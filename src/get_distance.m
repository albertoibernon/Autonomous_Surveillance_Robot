function [distancia_medida,closest,angulo] = get_distance(punto,p1,p2)

    % Calcular la distancia entre el robot y la recta del ransac 
    x1 = p1(1);
    y1 = p1(2);
    x2 = p2(1);
    y2 = p2(2);
    x0 = punto(1);
    y0 = punto(2);
    
    % Coeficientes de la recta que define la pared (Ax + By + C = 0)
    A = y2 - y1;
    B = x1 - x2;
    C = x2*y1 - x1*y2;
    
    % Distancia medida desde el punto a la recta
    distancia_medida = abs(A * x0 + B * y0 + C) / sqrt(A^2 + B^2);

    % Coordenadas del punto más cercano de la recta al punto donde está el robot
    closest = [(B*(B*x0 - A*y0) - A*C)/(A^2 + B^2), (A*(A*y0 - B*x0) - B*C)/(A^2 + B^2)];

    % Coeficientes de la recta definida entre el punto del robot y el punto
    % de la pared más cercano
    A1 = closest(2) - y0;
    B1 = x0 - closest(1);
    dir_recta1 = [A, B];
    dir_recta2 = [A1, B1];

    % Obtenemos el ángulo que forman las dos rectas 
    cos_theta = dot(dir_recta1, dir_recta2) / (norm(dir_recta1) * norm(dir_recta2));
    angulo = acos(cos_theta);

    if abs(angulo) > pi/2
        angulo = pi-abs(angulo);
    else
        angulo=abs(angulo);
    end

end