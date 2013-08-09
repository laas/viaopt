function dq = ThetaNoPtrb(t,q,alpha,k,T,cible)

dq = zeros(4,1);    % a column vector

% masse volumique : alu
rho = 2700;
% raideur du ressort
% k = 10000;

% longueurs caract�ristiques syst�me
L = 0.2;
l = 0.1;
S = 0.05^2;

% Coeff inertie
M = rho* L^3 * S/3;

% Coeff equa diff normalisee
A = k*l^2/M;


dq(1) = q(2);
dq(2) = -A*sin(q(1)+cible) - alpha;
if(T>=0.5)
    dq(2) = -A*sin(q(1)+cible) - alpha +0*k*L^2/M/10*sin(q(3)-q(1));
end;

dq(3) = q(4);
dq(4) = alpha;
