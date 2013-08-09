function dq = theta(q,t,u)

% parameters
k = 10000;
L = 0.2;
l = 0.1;
S = 0.05^2;
rho = 2700;
J = rho*S*L^2;
A = k*l*l/J;

dq(1,1) = q(2,1);
dq(2,1) = -A*q(1,1) - u;
dq(3,1) = q(4,1);
dq(4,1) = u;