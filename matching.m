function idx1,idx2=matching(cell1, cell2)  

    % Inicializar matrices para almacenar las diferencias
    diffDistancias = zeros(numel(cell1), numel(cell2));
    diffAngulos = zeros(numel(cell1), numel(cell2));

    % Calcular las diferencias entre distancias y ángulos
    for i = 1:numel(cell1)
        for j = 1:numel(cell2)
            diffDistancias(i, j) = abs(cell1{i}(1) - cell2{j}(1));
            diffAngulos(i, j) = abs(cell1{i}(2) - cell2{j}(2));
        end
    end

    % Encontrar la combinación que minimiza el error total
    [minDiff, idx] = min(diffDistancias + diffAngulos, [], 'all');
    [idx1, idx2] = ind2sub(size(diffDistancias), idx);

    % Mostrar los resultados
    % for k = 1:numel(cell1)
    %     fprintf('Celda %d de cell1 con Celda %d de cell2\n', k, idx2(k));
    % end
end
