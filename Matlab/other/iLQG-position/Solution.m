function X = Solution(n,dT,u)

x0 = [0;0;0;0];
X = x0;

for i = 1:n-1
    [t,Xsol] = ode45(@(t,x)Theta(t,x,u),[(i-1)*dT i*dT],x0);
    [n,p] = size(t);
    x0 = Xsol(n,:)';
    X = [X x0];
%     Couple = k*u2^2*sin(x0(1,1));
%     plot(i*dT,Couple,'g+'); %,i*dT,x0(1,1),'b',i*dT,u2,'r');
%     hold on;
end;

X;