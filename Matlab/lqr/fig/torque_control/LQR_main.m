% Commande LQR sans contrôle de la position réelle


dT = 0.0001;   % pas de discrétisation
n = 10000;       % nombre de pas


% masse volumique : alu
rho = 2700;
% raideur du ressort
 k = ones(n,1)*10000;
%dk = (10000-5000)/(n-1);
%k = [5000:dk:10000];

% longueurs caractéristiques système
L = 0.2; % longueur totale du bras
l = 0.1; % position du point d'application
S = 0.05^2; % section du bras

% Coeff inertie
J = rho* L^3 * S/3; 
% K = k*l*l/J; % pulsation propre au carré


% A = dT*[0 1 0 0; -K 0 0 0; 0 0 0 1; 0 0 0 0]+ eye(4);
B = dT*[0;-1;0;1];
% Q = [1 0 0 0;0 dT^2*10 0 0; 0 0 1 0; 0 0 0 dT^2];
Q1 = 0.001*[1 0 0 0;0 dT^2 0 0; 0 0 100 0; 0 0 0 10*dT^2];
Q2 = 1*[1 0 0 0;0 dT^2 0 0; 0 0 0.01 0; 0 0 0 0.1*dT^2];
R = dT^4*10000;
Pk = Q2;
Ptot = Q2;

% Détermination des Pk
for i = n-1:-1:0
    
    % Changement de matrice de poids Q au cours du temps
    if (i>0.5*n)
        Q=Q2;
    else
        Q=Q2;
    end
    
    K = k(i+1)*l*l/J;
    A = dT*[0 1 0 0; -K 0 0 0; 0 0 0 1; 0 0 0 0]+ eye(4);
    
    Pkk = Pk;
    Mkk = Pkk - Pkk*B*B'*Pkk/(R + B'*Pkk*B);
    Pk = Q + A'*Mkk*A;
    Ptot = [Pk;Ptot]; % Stockage des Pk dans une matrice Ptot
end

% Commande en position
% X0 = [0;0;-0.2;0]; % Vecteur d'état initial
%Commande en couple
X0 = [-pi/6;0;0;0];

Xkk = X0;
Xtot = X0;

%plot(0,0,'g+');
hold on;

for j = 0:n-1 % Détermination des Xk
    Xk = Xkk;
    
%     for m = 1:4
%         Pkk(m,:) = Ptot(4*j+m,:);
%     end

    Pkk = Ptot(4*j+1:4*j+4,:); 
    
    Xkk = (A - B*B'*Pkk/R)*Xk;
    Xtot = [Xtot Xkk];
    
    % affichage de qA et qB
    % plot((j+1)*dT,Xkk(3,1)+0.2,'b*',(j+1)*dT,Xkk(3,1)+Xkk(1,1)+0.2,'m+',(j+1)*dT,Xkk(1,1),'g'); 
    
    % affichage du couple
    couple = k(j+1)*L^2*sin(Xkk(1)+pi/6);
    plot((j+1)*dT,couple,'b+');
    hold on;
    
end

% Commande en positio
% legend('qA','qB','Theta');
% xlabel('Time (s)');
% ylabel('Angular Position (Rd)');
% title ('Ideal Case - Variation of k, from 5000 Nm-1 to 10000 Nm-1');

% Commande en couple
legend('Torque en B');
xlabel('Time (s)');
ylabel('Torque (N)');
title ('Ideal Case');


% Calcul des Xk réels pour comparer avec les Xk idéaux de la commande
% linéarisée
% Utot= [];
% for j = 1:n
%     Xkk = Xtot(:,j+1);
%     Pkk = Ptot(4*j+1:4*j+4,:);
%     Uk = -B'*Pkk*Xkk/R;
%     Utot = [Utot; Uk];
% %     plot(j*dT,Uk,'+'); % Tracé de la commande
% 
%     
% % Résolution de l'équation différentielle
% 
%     U = 2/dT*dT*(Xtot(3,j+1) - Xtot(3,j) - Xtot(4,j)*dT)
%     [T,Xsol_reel] = ode45(@(t,q)ThetaNoPtrb(t,q,U,k(j)),[(j-1)*dT j*dT], X0);
%     [N,p] = size(T);
%     X0 = Xsol_reel(N,:)';
%     
%     plot((j*dT),X0(3,1)+0.2,'bo',j*dT,X0(3,1)+X0(1,1)+0.2,'md');
%     hold on;
% end