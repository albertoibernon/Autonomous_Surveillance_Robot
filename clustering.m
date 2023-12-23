function clustering(nubeDePuntos)
% nubeDePuntos: una matriz donde cada fila representa un punto (al menos dos columnas)

    % Parámetros de RANSAC
    numIteraciones = 1000;  % Número de iteraciones de RANSAC
    umbralDistancia = 0.1;  % Umbral de distancia para considerar inliers

    % Ejecutar RANSAC
    [modelo, inliers] = ransac(nubeDePuntos', 'line', 2, numIteraciones, umbralDistancia);

    % Mostrar resultados
    figure;
    scatter(nubeDePuntos(:, 1), nubeDePuntos(:, 2), 'b.');  % Todos los puntos en azul
    hold on;

    % Resaltar inliers en rojo
    scatter(nubeDePuntos(inliers, 1), nubeDePuntos(inliers, 2), 'r.');

    % Graficar la recta ajustada
    slope = modelo.Parameters(2) / modelo.Parameters(1);
    intercept = modelo.Parameters(4) - slope * modelo.Parameters(3);
    xlim = get(gca, 'XLim');
    plot(xlim, slope * xlim + intercept, 'k-', 'LineWidth', 2);

    title('Ajuste de Recta RANSAC');
    xlabel('X');
    ylabel('Y');
    legend('Todos los puntos', 'Inliers', 'Recta RANSAC', 'Location', 'Best');
    hold off;
end