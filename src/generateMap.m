function [map]  = generateMap(path, threshold, scale, side)
    
    %% Lectura de la imagen del mapa
    image = imread(path);

    %% Transformación a escala de grises
    grayimage = rgb2gray(image);

    %% Escalado del mapa
    grayimage = imresize(grayimage, [side*scale, side*scale]);
    
    %% Transformación a imagen binaria
    bwimage = 1 - double(grayimage)/255;
    bwimage = bwimage > threshold;

    %% Transformación de la imagen a un mapa de ocupación binaria
    map = binaryOccupancyMap(bwimage,scale);
    
end