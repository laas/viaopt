function [Xreel,XBruit,T0] =  functionLQR(X0,XBruit,Ptot,T0,dT,n,pas,A,B,R,cible,k,L)
% fonction permettant de calculer les Xk réels dans le cas LQR à horizon
% fini

Xkk = XBruit;
Xtot = XBruit;

for j = 0:pas-1
    Xk = Xkk;

    Pkk = Ptot(4*j+1:4*j+4,:);
    
    Xkk = (A - B*B'*Pkk/R)*Xk;
    Xtot = [Xtot Xkk];
end


Utot= [];
for j = 1:pas
    T0 = T0+dT;
    Xkk = Xtot(:,j+1);
    Pkk = Ptot(4*j+1:4*j+4,:);
    Uk = -B'*Pkk*Xkk/R;
    Utot = [Utot; Uk];
%     plot(j*dT,Uk,'+');
    
    [T,Xsol_reel] = ode45(@(t,q)ThetaNoPtrb(t,q,Uk,k),[(j-1)*dT j*dT], X0);
    [N,p] = size(T);
    X0 = Xsol_reel(N,:)';
    % Commande en position
    % plot((T0),X0(3,1)+cible,'b+',T0,X0(3,1)+X0(1,1)+cible,'m*',T0,X0(1,1),'g');
   
    % affichage du couple
    couple = k*L^2*sin(X0(1)+pi/6);
    plot(T0,couple,'b+');
    hold on;
end


% Bruit blanc
BruitPcp = [0.01; 1.5 ; 0.2 ; 4.5];

BruitInstant = 2*(0.5-rand) * 0.1*BruitPcp;

Xreel = X0;
XBruit = X0 + BruitInstant;