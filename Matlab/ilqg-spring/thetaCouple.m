function dq = thetaCouple(t,q,u1,u2)

% parameters
k = 10000;
L = 0.2;
l = 0.1;
S = 0.05^2;
rho = 2700;
J = rho*S*L^2;
A = k*l*l/J;

dq(1,1) = q(2,1);
dq(2,1) = -k/J*q(3,1)^2*sin(q(1,1)) - u1;
dq(3,1) = q(4,1);
dq(4,1) = u2;
dq(5,1) = q(6,1);
dq(6,1) = -u1;