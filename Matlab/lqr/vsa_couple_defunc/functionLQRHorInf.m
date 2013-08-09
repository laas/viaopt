function [Xreel,XBruit,T0] =  functionLQRHorInf(X0,XBruit,P,G,T0,dT,n,pas,A,B,R,cible,k)

% fonction calculant la position réelle dans le cas LQR à horizon infini

Xk = X0;
Xtot = X0;

% for j = 0:pas-1   
%     Xk = (eye(6) - B*inv(R)*B'*P)*A*Xk;
%     Xtot = [Xtot Xk];
% end


Utot= [];
for j = 1:pas
    T0 = T0+dT;
    Uk = -G*(eye(6) - B*inv(R)*B'*P)*A*Xk;

    [T,Xsol_reel] = ode45(@(t,q)ThetaNoPtrb(t,q,Uk(1),Uk(2),k,cible),[(j-1)*dT j*dT], X0);
    [N,p] = size(T);    
    X0 = Xsol_reel(N,:)';
    
    % Commande en position
    % plot((T0),X0(3,1)+cible,'b+',T0,X0(3,1)+X0(1,1)+cible,'m*',T0,X0(1,1),'g');
    
    % affichage du couple
    couple = k*X0(5,1)^2*sin(X0(1,1)+cible);
    subplot(1,2,1);
    plot(T0,couple,'b*');
    hold on;
    
    subplot(1,2,2);
    plot(T0,X0(5,1),'g+');
    hold on;
    
end

% Bruit blanc

Xreel = X0;

% BruitPcp = [0.01; 1.5 ; 0.2 ; 4.5];
% BruitInstant = 2*(0.5-rand) * 0.05*BruitPcp;
% XBruit = X0 + 0*BruitInstant;