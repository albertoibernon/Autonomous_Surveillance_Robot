function coordenadasCartesianas = polaresACartesianas(coordenadasPolares)
    % coordenadasPolares: una matriz de dos columnas, donde la primera columna es r (radio)
    % y la segunda columna es theta (ángulo en radianes)

    % Extraer las columnas de radio y ángulo
    r = coordenadasPolares(1,:);
    theta = coordenadasPolares(2,:);

    % Convertir a coordenadas cartesianas
    x = r.*cos(theta); % offset = 135 deg
    y = r.*sin(theta); % offset = 135 deg

    % Combinar las coordenadas x e y en una matriz de coordenadas cartesianas
    coordenadasCartesianas = cat(1,x,y);
end