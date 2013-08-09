function dP = LQR(t,Pcol)

dT = 1;
dP = zeros(4,4);
R = 10;

P = [Pcol(1:4,1) Pcol(5:8,1) Pcol(9:12,1) Pcol(13:16,1)];

% masse volumique : alu
rho = 2700;
% raideur du ressort
k = 10000;

% longueurs caract�ristiques syst�me
L = 0.2;
l = 0.1;
S = 0.05^2;

% Coeff inertie
M = rho* L^3 * S/3;

% Coeff equa diff normalisee
delta = sqrt(k*l^2/M);

A = [0 1 0 0;-delta^2 0 0 0 ; 0 0 0 1 ; 0 0 0 0];
C = [0 0 0 0;0 1 0 1;0 0 0 0;0 1 0 1]/R;
% S = [1 0 0 0;0 dT^2 0 0;0 0 1 0;0 0 0 dT^2];


if(t<0.8)
    Interm = -P*A - A'*P + P*C*P;
else
    Interm = -P*A - A'*P + P*C*P-1000*eye(4);
end

dP = [Interm(:,1);Interm(:,2);Interm(:,3);Interm(:,4)];
