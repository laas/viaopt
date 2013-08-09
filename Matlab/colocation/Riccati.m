
% masse volumique : alu
rho = 2700;
% raideur du ressort
k = 10000;

% longueurs caract�ristiques syst�me
L = 0.2;
l = 0.1;
s = 0.05^2;

% Coeff inertie
M = rho* L^3 * s/3;

% Coeff equa diff normalisee
delta = sqrt(k*l^2/M);

% matrices
% R = 10;
R = eye(4);
S = 1*[100 0 0 0 ; 0 10 0 0 ; 0 0 100 0 ; 0 0 0 100]';
A = [0 1 0 0;-delta^2 0 0 0 ; 0 0 0 1 ; 0 0 0 0];
% B = [0;1;0;1];
B = [ 0 0 0 0 ; 1 0 0 0 ; 0 0 0 0 ; 1 0 0 0 ];
Q = eye(4,4);

[P,L,G] = care(A,B,Q,R,S);
