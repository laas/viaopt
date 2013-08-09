function dx = Theta(t,x,u1,u2)

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


% --- Theta
dx(1,1) = x(2,1);
dx(2,1) = -K*u2^2*x(1,1) - u1;