function dq = ThetaNoPtrb(t,q,alpha,beta)

dq = zeros(4,1);    % a column vector

% masse volumique : alu
rho = 2700;
% raideur du ressort
k = 10000;

% longueurs caract�ristiques syst�me
L = 0.2;
l = 0.1;
S = 0.05^2;

% Coeff inertie
J = rho* L^3 * S/3;

% Coeff equa diff normalisee
K = k*q(5)^2/J;


dq(1) = q(2);
dq(2) = -K*(sin(q(1))) - alpha;

dq(3) = q(4);
dq(4) = alpha;

dq(5) = q(6);
dq(6) = beta;

if((q(5)>=0.15 && beta>=0) ||(q(5)<=0.05 && beta<=0))
    dq(5) = 0;
    dq(6) = -beta;
end;
        