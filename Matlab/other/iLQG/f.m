function X = f(x,u,dT)



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
X(1,1) = x(1,1) + dT*x(2,1);
X(2,1) = x(2,1) + dT*(-K*u(2,1)^2*x(1,1) - u(1,1));
% 
% % --- qA
% X(3,1) = x(3,1) + dT*x(4,1);
% X(4,1) = x(4,1) + dT*u(1,1);

% --- r
% X(3,1) = x(3,1) + dT*x(4,1);
% X(4,1) = x(4,1) + dT*u(2,1);