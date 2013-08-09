function [fx fu] = DeriveesF(xi,ui,dT)

% parameters
k = 10000;
L = 0.2;
l = 0.1;
S = 0.05^2;
rho = 2700;
J = rho*S*L^2;
K = k*l*l/J;

fx = [1 dT 0 0;-dT*k/J*xi(3,1)^2 1 -2*dT*k/J*xi(3,1)*xi(1,1) 0;0 0 1 dT;0 0 0 1];

fu = [0 0;-dT 0;0 0;0 dT];
% 
% fx = [1 dT;-dT*k/J*ui(2,1)^2 1];
% 
% fu = [0 0;-dT -2*dT*k/J*ui(2,1)*xi(1,1)];