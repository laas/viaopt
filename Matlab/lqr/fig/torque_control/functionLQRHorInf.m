function [Xreel,XBruit,T0] =  functionLQRHorInf(X0,XBruit,P,G,T0,dT,n,pas,A,B,R,cible,k,l,ALPHA)
% fonction calculant la position réelle dans le cas LQR à horizon infini

Xk = XBruit;
Xtot = XBruit;

for j = 0:pas-1   
    Xk = (A - B*B'*P/R)*Xk;
    Xtot = [Xtot Xk];
end


Utot= [];
for j = 1:pas
    T0 = T0+dT;
    Xk = Xtot(:,j);
    Uk = -G*Xk;
    Utot = [Utot; Uk];
%     plot(j*dT,Uk,'+');
    
% if(T0<=0.5)
%     U = ALPHA*exp(T0-0.5)^5;
%     %[T,Xsol_reel] = ode45(@(t,q)ThetaNoPtrb(t,q,ALPHA*exp(T0-0.5)^5,k,T0,cible),[(j-1)*dT j*dT], X0);
% else
%     %[T,Xsol_reel] = ode45(@(t,q)ThetaNoPtrb(t,q,ALPHA,k,T0,cible),[(j-1)*dT j*dT], X0);
% end;


    [T,Xsol_reel] = ode45(@(t,q)ThetaNoPtrb(t,q,Uk,k,T0,cible),[(j-1)*dT j*dT], X0);
    [N,p] = size(T);    
    X0 = Xsol_reel(N,:)';
    
    % Commande en position
    % plot((T0),X0(3,1)+cible,'b+',T0,X0(3,1)+X0(1,1)+cible,'m*',T0,X0(1,1),'g');
    
    % affichage du couple
    %subplot(1,2,1);
    couple = k*l^2*sin(X0(1)+cible);
    plot(T0,couple,'b-');
    hold on;
    
%     subplot(1,2,2);
%     plot(T0,X0(1)+cible,'g',T0,X0(3),'r');
%     hold on;
    
end

% Bruit blanc
BruitPcp = [0.01; 1.5 ; 0.2 ; 4.5];

BruitInstant = 2*(0.5-rand) * 0.05*BruitPcp;

Xreel = X0;
XBruit = X0 + 0*BruitInstant;