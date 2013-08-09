function [X U] = Initialisation(n,dT,Tcible,r,X,U)

mu = 5e-2;
alpha = 0.1;
iter = 15;

%X = zeros(4,n);
%U = zeros(1,n);
raid = 10000;

HX = [];
HU = [];

% poids
I = diag([100,dT^2,100,dT^2]);
V = 0;

for l = 1:iter
 
k = [];
K =[];

xN = X(:,n);
VX = 1000*[2*raid*r^2*(raid*r^2*xN(1,1)-Tcible);0;0;0];
VXX = 1000*diag([2*raid^2*r^4,0,0,0]);
Vnouv = (raid*r^2*xN(1,1) -Tcible)^2;

for j = n-1:-1:1
    
    xi = X(:,j);
    ui = U(:,j);
    
    Vx = VX(:,1);
    Vxx = VXX(:,1:4);
    
%     Lx = LX(xi,ui,I,dT);
%    Lu = LU(xi,ui,I);
%    Lxx = LXX(xi,ui,I,dT);
%    Luu = LUU(xi,ui,I);
 
[Lx Lu Lxx Luu Lux Lxu] = DeriveesLInit(xi,ui,Tcible);

    fx = FXInit(xi,ui,dT);
    fu = FUInit(xi,ui,dT);
    
    Qx = Lx + fx'*Vx;
    Qu = Lu + fu'*Vx;
    Qxx = Lxx + fx'*(Vxx+diag([mu,mu,0,0]))*fx;
    Quu = Luu + fu'*(Vxx+diag([mu,mu,0,0]))*fu;
    Qux = fu'*(Vxx+diag([mu,mu,0,0]))*fx;
    Qxu = fx'*(Vxx+diag([mu,mu,0,0]))*fu;
    
    ki = -inv(Quu)*Qu;
    Ki = -inv(Quu)*Qux;
    
    k = [ki k];
    K = [Ki K];
    
    Vxi = Qx - Qux'*inv(Quu)*Qu;
    Vxxi = Qxx - Qxu*inv(Quu)*Qux;
    
    VX = [Vxi VX];
    VXX = [Vxxi VXX];
    
end;

Xt = X(:,1);
Ut = [];

for i = 1:n-1
    Uti = U(:,i) + alpha*k(:,i) + K(:,4*i-3:4*i)*(Xt(:,i) - X(:,i));
    Xtii = fInit(Xt(:,i),Uti,dT);
    Xt = [Xt Xtii];
    Ut = [Ut Uti];
    
    dx =  -(Xt(:,i) - X(:,i));
    Vx = VX(:,i);
    Vxx = VXX(:,4*i-3:4*i);
    Vnouv = Vnouv + Vx'*dx + 1/2* dx'*Vxx*dx;
    
end;

X = Xt;
U = Ut;

if (l==1)
    Vanc = Vnouv;
end;
  
if (abs(Vanc)>abs(Vnouv))
    Vanc = Vnouv;
end;

end;




