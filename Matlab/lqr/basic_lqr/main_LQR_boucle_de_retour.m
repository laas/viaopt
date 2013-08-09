% masse volumique : alu
rho = 2700;
% raideur du ressort
k = 1.0*10000;

% longueurs caract�ristiques syst�me
L = 0.2;
l = 0.1;
S = 0.05^2;

% Coeff inertie
J = rho* L^3 * S/3;
K = k*l*l/J; % pulsation propre au carré


dT = 0.001;   % pas de discrétisation
n = 2000;       % nombre de pas total
pas = 2;        % nombre de pas avant correction
cible = 0.2;

A = dT*[0 1 0 0; -K 0 0 0; 0 0 0 1; 0 0 0 0]+ eye(4);
B = dT*[0;-1;0;1];
Q1 = 0.0*[100 0 0 0;0 dT^2*10 0 0; 0 0 1 0; 0 0 0 dT^2];
Q2 = 10*[1 0 0 0;0 dT^2 0 0; 0 0 1 0; 0 0 0 dT^2];
R = dT^4*100;


Pk = Q2;
Ptot = Q2;

for i = n-1:-1:0
    
    if (i>0.1*n)
        Q=Q1;
    else
        Q=Q2;
    end
    
    Pkk = Pk;
    Mkk = Pkk - Pkk*B*B'*Pkk/(R + B'*Pkk*B);
    Pk = Q + A'*Mkk*A;
    Ptot = [Pk;Ptot];
end

X0 = [0;0;-cible;0];
T = 0;

for i = 1:500
    [X0,T] = functionLQR(X0,Ptot,T,dT,n,pas,A,B,R,cible);   
end

title = ('Commande LQR, avec k biaisé de 50%');
xlabel('time (s)');
ylabel('Angular Position (rd)');
legend('qA avec biais','qB avec biais','qA sans biais','qB sans biais');
