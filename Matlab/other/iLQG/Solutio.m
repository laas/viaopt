function X = Solutio(M,dT)

x0 = [0;0];
X = x0;
dT = 0.05;

% masse volumique : alu
rho = 2700;
% raideur du ressort
k = 100000;

% longueurs caract�ristiques syst�me
L = 0.2;
l = 0.1;
S = 0.05^2;

% Coeff inertie
J = rho* L^3 * S/3;


u1 = -100/J;
u2 = 0.1;

for i = 1:M
    [t,Xsol] = ode45(@(t,x)Theta(t,x,u1,u2),[(i-1)*dT i*dT],x0);
    [n,p] = size(t);
    x0 = Xsol(n,:)';
    X = [X x0];
%     Couple = k*u2^2*sin(x0(1,1));
%     plot(i*dT,Couple,'g+'); %,i*dT,x0(1,1),'b',i*dT,u2,'r');
%     hold on;
end;

X;