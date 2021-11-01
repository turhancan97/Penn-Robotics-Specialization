% In this problem, we'll use the example of writing a function to create a
% rotation matrix to show how to complete programming assignments. This is
% an ungraded problem and is optional.

% Rotation matrices are widely used in many fields, including robotics. Our
% task is to write a function that accepts an angle in radians (as a scalar
% value) and returns a 3x3 matrix for the rotation matrix about the z-axis.

function Rz = zrot( phi )

Rz = [cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];

end