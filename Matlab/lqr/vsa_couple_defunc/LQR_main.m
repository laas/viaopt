% Commande LQR sans contrôle de la position réelle


dT = 0.001;   % pas de discrétisation
n = 1000;       % nombre de pas


% masse volumique : alu
rho = 2700;
% raideur moyenne du ressort
k = 10000;

% longueurs caractéristiques système
L = 0.2; % longueur totale du bras
l = 0.1; % position du point d'application -  r moyen
S = 0.05^2; % section du bras

% Coeff inertie
J = rho* L^3 * S/3; 
K = k*l*l/J; % pulsation propre au carré


A = dT*[0 1 0 0 0 0; -K 0 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0;0 0 0 0 0 1;0 0 0 0 0 0]+ eye(6);
B = dT*[0 0;-1 -k/J*l*sin(0.01);0 0;1 0;0 0;0 1];
Q = [1e8 0 0 0 0 0;0 1e5*dT^2 0 0 0 0; 0 0 1e-4 0 0 0; 0 0 0 1e-4*dT^2 0 0;0 0 0 0 1 0;0 0 0 0 0 dT^2];
R = dT^4*100*eye(2);
Pk = Q;
Ptot = Q;

% Détermination des Pk
for i = n-1:-1:0
       
    Pkk = Pk;
    Mkk = Pkk - Pkk*B*inv((R + B'*Pkk*B))*B'*Pkk;
    Pk = Q + A'*Mkk*A;
    Ptot = [Pk;Ptot]; % Stockage des Pk dans une matrice Ptot
end

% Commande en position
% X0 = [0;0;-0.2;0]; % Vecteur d'état initial
%Commande en couple avec VSA
cible = asin(200/(k*0.05^2));
X0 = [-cible;0;0;0;0.1;0];

Xkk = X0;
Xtot = X0;

%plot(0,0,'g+');
hold on;

for j = 0:n-1 % Détermination des Xk
    Xk = Xkk;
    
%     for m = 1:4
%         Pkk(m,:) = Ptot(4*j+m,:);
%     end

    Pkk = Ptot(4*j+1:4*j+6,:); 
    
    Xkk = (eye(6) - B*inv(R)*B'*Pkk)*A*Xk;
    Xtot = [Xtot Xkk];
    
    % affichage de qA et qB
    % plot((j+1)*dT,Xkk(3,1)+0.2,'b*',(j+1)*dT,Xkk(3,1)+Xkk(1,1)+0.2,'m+',(j+1)*dT,Xkk(1,1),'g'); 
    
    % affichage du couple
    couple = k*Xkk(5,1)^2*sin(Xkk(1,1)+cible);
    plot((j+1)*dT,couple,'b+');
    hold on;
    
end


% Commande en couple
legend('Torque en B');
xlabel('Time (s)');
ylabel('Torque (N)');
title ('Ideal Case');

