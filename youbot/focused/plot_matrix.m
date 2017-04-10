% A map can be represented by a matrix of integers. Here, the convention is the following: 
%   -  0 for accessible cells
%   -  1 for obstacles
% The type of the elements is chosen to be logical (as in Fortran; this type is more commonly called bool or boolean).
% This allows to use as little memory as possible for the same map size (and thus to have faster operations on the map).
% To store larger elements, you could use 'uint8' (unsigned integer, 8 bits) or 'int8' (signed integer, 8 bits). 
map = ones(64, 64, 'logical'); % At first, no cell is accessible. 
map(20:40, 20:40) = 0; % Dig a rectangle in it. 
map(30, 25:35) = 1; % Draw three walls. 
map(25:35, [25, 35]) = 1;

% With this representation, the contour function can be used to draw the map. 
figure('name', 'contour'); 
contour(map);

% imagesc can also be used for this purpose. 
figure('name', 'imagesc'); 
imagesc(map); 