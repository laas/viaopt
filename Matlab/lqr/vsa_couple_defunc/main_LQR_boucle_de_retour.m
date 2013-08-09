clear all;

% masse volumique : alu
rho = 2700;
% raideur du ressort
 k = 80000;

% longueurs caract�ristiques syst�me
L = 0.2;
l = 0.1;
S = 0.05^2;

% Coeff inertie
J = rho* L^3 * S/3;
 K = k*l*l/J; % pulsation propre au carré


dT = 0.001;   % pas de discrétisation
n = 1000;       % nombre de pas total
pas = 1;        % nombre de pas avant correction
cible = 0.2;

A = dT*[0 1 0 0 0 0; -K 0 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0;0 0 0 0 0 1;0 0 0 0 0 0]+ eye(6);
B = dT*[0 0;-1 -k/J*l*sin(0.01);0 0;1 0;0 0;0 1];
Q2 = 0.0001*[10 0 0 0 0 0;0 dT^2 0 0 0 0; 0 0 1 0 0 0; 0 0 0 dT^2 0 0;0 0 0 0 1 0;0 0 0 0 0 0.00001*dT^2];
Q1 = 0.00001*[10 0 0 0 0 0;0 dT^2 0 0 0 0; 0 0 1 0 0 0; 0 0 0 dT^2 0 0;0 0 0 0 1 0;0 0 0 0 0 0.00001*dT^2];
R = dT^4*100000*[1 0;0 1];
Pk = Q2;
Ptot = Q2;


Pk = Q2;
Ptot = Q2;

for i = n-1:-1:0
    
    if (i>0.1*n)
        Q=Q2;
    else
        Q=Q1;
    end
    
    Pkk = Pk;
    Mkk = Pkk - Pkk*B*inv((R + B'*Pkk*B))*B'*Pkk;
    Pk = Q + A'*Mkk*A;
    Ptot = [Pk;Ptot];
end

% Commande en position
% X0 = [0;0;-0.2;0];% Vecteur d'état initial
XBruit = [0;0;0;0;0;0];

%Commande en couple
cible = asin(200/(k*0.05^2));
X0 = [-cible;0;0;0;0.1;0];
T =0;

for i = 1:n
    
    [X0,XBruit,T] = functionLQR(X0,XBruit,Ptot,T,dT,n,pas,A,B,R,cible,k,L);  
    
end;


