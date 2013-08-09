function dq = Theta(t,q,u)

% masse volumique : alu
rho = 2700;
% raideur du ressort
k = 100000;

% longueurs caract�ristiques syst�me
L = 0.2;
l = 0.1;
S = 0.05^2;

% Coeff inertie
J = rho* L^3 * S/3;

dq(1,1) = q(2,1);
dq(2,1) = -k*l^2/J*q(1,1) - u;
dq(3,1) = q(4,1);
dq(4,1) = u;