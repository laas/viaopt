% Commande LQR sans contrôle de la position réelle



% masse volumique : alu
rho = 2700;
% raideur du ressort
k = 10000;

% longueurs caractéristiques système
L = 0.2; % longueur totale du bras
l = 0.1; % position du point d'application
S = 0.05^2; % section du bras

% Coeff inertie
J = rho* L^3 * S/3; 
K = k*l*l/J; % pulsation propre au carré


dT = 0.001;   % pas de discrétisation
n = 1000;       % nombre de pas

A = dT*[0 1 0 0; -K 0 0 0; 0 0 0 1; 0 0 0 0]+ eye(4);
B = dT*[0;-1;0;1];
% Q = [1 0 0 0;0 dT^2*10 0 0; 0 0 1 0; 0 0 0 dT^2];
Q1 = 0.01*[100 0 0 0;0 dT^2*10 0 0; 0 0 1 0; 0 0 0 dT^2];
Q2 = 10*[1 0 0 0;0 dT^2 0 0; 0 0 1 0; 0 0 0 dT^2];
R = dT^4*100;
Pk = Q2;
Ptot = Q2;

% Détermination des Pk
for i = n-1:-1:0
    
    % Changement de matrice de poids Q au cours du temps
    if (i>0.1*n)
        Q=Q2;
    else
        Q=Q2;
    end
    
    
    Pkk = Pk;
    Mkk = Pkk - Pkk*B*B'*Pkk/(R + B'*Pkk*B);
    Pk = Q + A'*Mkk*A;
    Ptot = [Pk;Ptot]; % Stockage des Pk dans une matrice Ptot
end

X0 = [0;0;-0.2;0]; % Vecteur d'état initial
Xkk = X0;
Xtot = X0;

plot(0,0,'g+');
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
    plot((j+1)*dT,Xkk(3,1)+0.2,'g*',(j+1)*dT,Xkk(3,1)+Xkk(1,1)+0.2,'r+'); 
    hold on;
    
end


% Calcul des Xk réels pour comparer avec les Xk idéaux de la commande
% linéarisée
Utot= [];
for j = 1:n
    Xkk = Xtot(:,j+1);
    Pkk = Ptot(4*j+1:4*j+4,:);
    Uk = -B'*Pkk*Xkk/R;
    Utot = [Utot; Uk];
%     plot(j*dT,Uk,'+'); % Tracé de la commande

    
% Résolution de l'équation différentielle
    [T,Xsol_reel] = ode45(@(t,q)ThetaNoPtrb(t,q,Uk),[(j-1)*dT j*dT], X0);
    [N,p] = size(T);
    X0 = Xsol_reel(N,:)';
    
    plot((j*dT),X0(3,1)+0.2,'bo',j*dT,X0(3,1)+X0(1,1)+0.2,'md');
    hold on;
end