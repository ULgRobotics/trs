% Prepare a map to draw and a few other points. 
map = ones(64, 64, 'logical'); 
map(20:40, 20:40) = 0; 
map(30, 25:35) = 1;  
map(25:35, [25, 35]) = 1;

youbot = [21, 24];
nextPoint = [24, 24];

% Make the first plot, just the map. 
f1 = figure('name', 'contour'); 
contour(map);

% Make the second plot, just the map. 
f2 = figure('name', 'imagesc'); 
imagesc(map); 

% Then, go back to the first plot (contour) and add the youbot as a cross. 
% (Use breakpoints to see how things work, step by step.)
figure(f1); 
hold on; 
plot(youbot(1), youbot(2), '+');

% Add the next point on the other map, with a star. 
figure(f2); 
hold on; 
plot(nextPoint(1), nextPoint(2), '*');

% Bring both plots to parity. 
figure(f1); 
hold on; 
plot(nextPoint(1), nextPoint(2), '*');

figure(f2); 
hold on; 
plot(youbot(1), youbot(2), '+');

% Draw the line between the two points in each window. 
figure(f1); 
hold on; 
plot([youbot(1), nextPoint(1)], [youbot(2), nextPoint(2)]);

figure(f2); 
hold on; 
plot([youbot(1), nextPoint(1)], [youbot(2), nextPoint(2)]);
