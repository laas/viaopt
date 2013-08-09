% masse volumique : alu
rho = 2700;
% raideur du ressort
% k = 10000;

% longueurs caract�ristiques syst�me
L = 0.2;
l = 0.1;
S = 0.05^2;

% Coeff inertie
J = rho* L^3 * S/3;
% K = k*l*l/J; % pulsation propre au carré


dT = 0.001;   % pas de discrétisation
n = 1000;       % nombre de pas total
pas = 1;        % nombre de pas avant correction
cible = 0.2;

% A = dT*[0 1 0 0; -K 0 0 0; 0 0 0 1; 0 0 0 0]+ eye(4);
B = dT*[0;-1;0;1];
% Q1 = 0.01*[100 0 0 0;0 10*dT^2 0 0; 0 0 1 0; 0 0 0 dT^2];
% Q2 = 10*[1 0 0 0;0 dT^2 0 0; 0 0 1 0; 0 0 0 dT^2];
% R = dT^4*100;

Q = [1e5 0 0 0;0 0.1*dT^2 0 0; 0 0 1e-10 0; 0 0 0 1e-10*dT^2];
R = dT^4*1;

dk = (10000-5000)/(n-1);
%  k = [5000:dk:10000];
k = 10000*ones(n,1);

Pk = Q;
Ptot = Q;

for i = n-1:-1:0
    
    K = k(i+1)*l*l/J; % pulsation propre au carré
    A = dT*[0 1 0 0; -K 0 0 0; 0 0 0 1; 0 0 0 0]+ eye(4);

    
    Pkk = Pk;
    Mkk = Pkk - Pkk*B*B'*Pkk/(R + B'*Pkk*B);
    Pk = Q + A'*Mkk*A;
    Ptot = [Pk;Ptot];
end

% Commande en position
% X0 = [0;0;-0.2;0];% Vecteur d'état initial
XBruit = [0;0;0;0];

%Commande en couple
X0 = [-pi/6;0;0;0];

T = 0;
Theta = 0;
ThetaPrim = 0;
QPrim = 0;

for i = 1:n
    
    K = (k(i)+2*(0.5-rand)*5000)*l*l/J; % pulsation propre au carré
    A = dT*[0 1 0 0; -K 0 0 0; 0 0 0 1; 0 0 0 0]+ eye(4);
    [X0,XBruit,T] = functionLQR(X0,XBruit,Ptot,T,dT,n,pas,A,B,R,cible,k(i),L);  
    
% %   Max des valeurs d'etat pour calculer bruit 
%     if(X0(1,1)>Theta)
%     Theta = X0(1,1);
%     end;
%     
%     if(X0(2,1)>ThetaPrim)
%     ThetaPrim = X0(2,1);
%     end;
%     
%     if(X0(4,1)>QPrim)
%     QPrim = X0(4,1);
%     end;
    
end;

% Theta
% ThetaPrim
% QPrim
% 
% legend('qA','qB','Theta');
% xlabel('Time (s)');
% ylabel('Angular Position (Rd)');
% title ('Finite Horizon with Feedback- Variation of k and Random Interference on State and Stiffness');
% % title ('Finite Horizon with Feedback - Random Interference on the Stiffness');

% Commande en couple
legend('Torque en B');
xlabel('Time (s)');
ylabel('Torque (N)');
title ('With Feedback');
