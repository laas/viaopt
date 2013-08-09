function F = f(xi,ui,dT)

% parameters
k = 10000;
L = 0.2;
l = 0.1;
S = 0.05^2;
rho = 2700;
J = rho*S*L^2;
K = k*l*l/J;

% A = [0 1 0 0;-k*l^2/J 0 0 0;0 0 0 1;0 0 0 0];
% B = [0;-1;0;1];
% 
% F = (eye(4) + dT*A)*xi + dT*B*ui;

F = [xi(1,1) + xi(2,1)*dT; xi(2,1) + dT*(-k*xi(3,1)^2/J*xi(1,1) - ui(1,1));xi(3,1) + xi(4,1)*dT;xi(4,1) + dT*ui(2,1)];