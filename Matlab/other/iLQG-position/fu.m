function F = fu(xi,ui,dT)

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

% Coeff equa diff normalisee
K = k/J;

B = dT*[0;-1;0;1];

F = B;
