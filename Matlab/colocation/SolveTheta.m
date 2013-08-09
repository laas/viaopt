function [A,B,C,D] = SolveTheta(EtatPcdt,i,dT,alpha)

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

% resout la continuite a l'instant i
% syms A B;
% solT = solve('A*cos(i*dT/delta)+B*sin(i*dT/delta)+alpha/delta^2-Etatpcdt(1)','-A/delta*sin(i*dT/delta)+B/delta*cos(i*dT/delta)-Etatpcdt(2)',A,B);
% AA = (solT.A);
% BB = (solT.B);

B = (EtatPcdt(1)-alpha/delta^2)*sin(i*dT/delta) + (delta*EtatPcdt(2))*cos(i*dT/delta);
A = (EtatPcdt(1) - B*sin(i*dT/delta) - alpha/delta^2) /(cos(idT/delta));

D = EtatPcdt(4)-alpha*i*dT;
C = EtatPcdt(3)-alpha*(i*dT)^2/2-D*i*dT;


