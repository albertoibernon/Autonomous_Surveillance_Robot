function puntos_polares = cartesianasAPolares(coordenadasCartesianas)
    % Extraer las coordenadas x e y
    x = coordenadasCartesianas(1, :);
    y = coordenadasCartesianas(2, :);

    % Convertir a coordenadas polares
    [theta, rho] = cart2pol(x, y);

    % Crear la matriz de coordenadas polares
    puntos_polares = [rho, theta];
    
end