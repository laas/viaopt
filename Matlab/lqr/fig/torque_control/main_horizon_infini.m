clear all;

% masse volumique : alu
rho = 2700;
% raideur du ressort
k = 100000;

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

% dk = (10000-5000)/(n-1);
% % k = [5000:dk:10000];
% k = 10000*ones(n,1);

A = dT*[0 1 0 0; -K 0 0 0; 0 0 0 1; 0 0 0 0]+ eye(4);
B = dT*[0;-1;0;1];
% Q1 = 0.01*[100 0 0 0;0 10*dT^2 0 0; 0 0 1 0; 0 0 0 10*dT^2];
% Q2 = 10*[1 0 0 0;0 dT^2 0 0; 0 0 1 0; 0 0 0 dT^2];
% R = dT^4*100;
Q1 = 0.01*[1000 0 0 0;0 dT^2 0 0; 0 0 10 0; 0 0 0 10*dT^2];
Q = [1e11 0 0 0;0 0.001*dT^2 0 0; 0 0 1e-10 0; 0 0 0 1e-10*dT^2];
R = dT^4*1;


% [P,V,G] = dare(A,B,Q,R); % Résolution de l'équation de Ricatti

% Commande en position
% X0 = [0;0;-0.2;0];% Vecteur d'état initial
XBruit = [0;0;0;0];

%Commande en couple
cible = asin(200/k/l^2);
ALPHA = -K*sin(cible);
X0 = [-cible;0;0;0];  

T = 0;                   % instant initial


% boucle de résolution des Xk réels
 for i = 1:1000
     [P,V,G] = dare(A,B,Q,R); % Résolution de l'équation de Ricatti
     [X0,XBruit,T] = functionLQRHorInf(X0,XBruit,P,G,T,dT,n,pas,A,B,R,cible,k,l,ALPHA);   
 end

% 
% legend('qA','qB','Theta');
% xlabel('Time (s)');
% ylabel('Angular Position (Rd)');
% %title ('Infinite Horizon - Variation of k, from 5000 Nm-1 to 10000 Nm-1');
% %title ('Finite Horizon with Feedback - Random Interference on the Stiffness');
% title ('Finite Horizon with Feedback- Variation of k and Random Interference on State (5%) and Stiffness');
% %title ('Finite Horizon with Feedback - Random Interference on the State : 5%');

% Commande en couple
legend('Torque en B');
xlabel('Time (s)');
ylabel('Torque (N)');
% title ('Infinite Horizon with Sinusoidal Impact from T=0.5');

