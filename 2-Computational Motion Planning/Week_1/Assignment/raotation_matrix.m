% This code will run without modification as long as your function returns an 3x3 matrix.

% Create a vector to rotate and choose an angle
% Try out multiple vectors
vec = [1; 0; 0]; % do not change the name if you want to use the visualization code below.
phi = pi/4;

% Create the rotation matrix and apply it to the vector
Rz = zrot(phi)
rotVec = Rz*vec

% Visualize the rotation
plot3([0, 1.5], [0, 0], [0, 0], 'k') % x-axis
hold on % This command allows us to put multiple lines on the same axes
plot3([0, 0], [0, 1.5], [0, 0], 'k') % y-axis
plot3([0, 0], [0, 0], [0, 1.5], 'k') % z-axis
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
% Plot the original vector
plot3([0, vec(1)], [0, vec(2)], [0, vec(3)], 'b', 'LineWidth', 2);

% Plot the rotated vector
plot3([0, rotVec(1)], [0, rotVec(2)], [0, rotVec(3)], 'r', 'LineWidth', 2);
title('Original Vector = blue, Rotated Vector = red')