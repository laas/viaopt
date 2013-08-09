function [Xreel,T0] =  functionLQR(X0,Ptot,T0,dT,n,pas,A,B,R,cible)
% fonction permettant de calculer les Xk réels dans le cas LQR à horizon
% fini

Xkk = X0;
Xtot = X0;

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
    
    [T,Xsol_reel] = ode45(@(t,q)ThetaNoPtrb(t,q,Uk),[(j-1)*dT j*dT], X0);
    [N,p] = size(T);
    X0 = Xsol_reel(N,:)';
    plot((T0),X0(3,1)+cible,'g',T0,X0(3,1)+X0(1,1)+cible,'r');
    hold on;
end

Xreel = X0;