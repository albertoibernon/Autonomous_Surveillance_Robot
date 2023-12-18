function [map]  = generateMap(path, threshold, scale)


image = imread(path);
%Convert image into grayscale.
grayimage = rgb2gray(image);
% imshow(grayimage);

% Threshold  image in order to get a binary image.
bwimage = 1 - double(grayimage)/255;
bwimage = bwimage > threshold;
% Convert image into  binaryOccupancyMap  with scale  (pixes/meter).
map = binaryOccupancyMap(bwimage,scale);

end