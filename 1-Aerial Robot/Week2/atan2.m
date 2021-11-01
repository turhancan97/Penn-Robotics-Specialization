%% atan2
% atan(arctangent)function
atan(0.8)
x = [0.5i 1+3i -2.2+i];
Y = atan(x)
%% 
x = -20:0.01:20; 
plot(x,atan(x))
grid on
%% 
% atan2 Four-quadrant inverse tangent
atan2(4,-3)
z = 4 + 3i;
r = abs(z)
theta = atan2(imag(z),real(z))