%% Symbolic Calculations in Matlab
syms a b real
x = (a+b)^2
expand(x)
y = (a/2)^2/a
z = sin(a)^2+cos(a)^2
simplify(z)
%%
A = [a 0 b; 0 -a 0; 0 1 0];
B = [2*a b exp(a); cos(a) 0 0; -a b/a 0];
A * B
A'
[V,D] = eig(A)
%%
syms phi theta psi real
R1 = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
R2 = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
R3 = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
simplify(R1*R2*R3)