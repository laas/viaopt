function X = Xreel(X0,U,dT,n,m)

raid = 10000;

X = [];
T0 = n*dT*(m-1);

for i = 1:n
u1 = U(1,i);
u2 = U(2,i);

[T,Xsol] = ode45(@(t,q)thetaCouple(t,q,u1,u2),[T0+(i-1)*dT T0+i*dT],X0);
[n,p] = size(T);
X0 = Xsol(n,:)';
X = [X X0];
    
% Couple = raid*Xsol(:,3).^2.*Xsol(:,1);
% plot(T,Couple,'b');
% hold on;

end;

