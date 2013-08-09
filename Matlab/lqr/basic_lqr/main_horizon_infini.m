% masse volumique : alu
rho = 2700;
% raideur du ressort
k = 10000;

% longueurs caractéristiques du système
L = 0.2;
l = 0.1;
S = 0.05^2;

% Coeff inertie
J = rho* L^3 * S/3;
K = k*l*l/J;    % pulsation propre au carré


dT = 0.001;     % pas de discrétisation
n = 1000;       % nombre de pas total
pas = 1;        % nombre de pas avant correction
cible = 0.2;    % position ciblée

A = dT*[0 1 0 0; -K 0 0 0; 0 0 0 1; 0 0 0 0]+ eye(4);
B = dT*[0;-1;0;1];
Q = 10*[1 0 0 0;0 dT^2 0 0; 0 0 1 0; 0 0 0 dT^2];
R = dT^4*100;

[P,V,G] = dare(A,B,Q,R); % Résolution de l'équation de Ricatti

X0 = [0;0;-cible;0];     % condition initiale
T = 0;                   % instant initial


% boucle de résolution des Xk réels
 for i = 1:1000
     [X0,T] = functionLQRHorInf(X0,P,G,T,dT,n,pas,A,B,R,cible);   
 end


