function [LX LU LXX LUU LUX LXU] = DeriveesLInit(xi,ui,Tcible)

% parameters
k = 10000;
L = 0.2;
l = 0.1;
S = 0.05^2;
rho = 2700;
J = rho*S*L^2;
K = k*l*l/J;

LX = [2*k*l^2*(k*l^2*xi(1,1)-Tcible);0;0;0];
LXX = diag([2*k^2*l^4,0,0,0]);

LU = 0*2*ui;
LUU = 0*2;

LUX = 0;
LXU = 0;
