function [Xreel,T0] =  functionLQRHorInf(X0,P,G,T0,dT,n,pas,A,B,R,cible)

% fonction calculant la position réelle dans le cas LQR à horizon infini

Xk = X0;
Xtot = X0;

for j = 0:pas-1   
    Xk = (eye(6) - B*inv(R)*B'*P)*A*Xk;
    Xtot = [Xtot Xk];
end


Utot= [];
for j = 1:pas
    T0 = T0+dT;
    Xk = Xtot(:,j);
    Uk = -G*Xk;
    Utot = [Utot; Uk];
%     plot(j*dT,Uk,'+');
    
    [T,Xsol_reel] = ode45(@(t,q)ThetaNoPtrb(t,q,Uk(1),Uk(2)),[(j-1)*dT j*dT], X0);
    [N,p] = size(T);
    X0 = Xsol_reel(N,:)';
    plot((T0),X0(3,1)+cible,'b-',T0,X0(3,1)+X0(1,1)+cible,'m-.',T0,X0(5,1),'g');
    hold on;
end

Xreel = X0;

