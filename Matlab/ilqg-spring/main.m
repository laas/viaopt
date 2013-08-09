n =200;
dT = 0.001;
Cible = 0.3;
Tcible = 20;
iter = 10;
mu = 0;
alpha = 0.1;

X = zeros(4,n); 
X(1:2,:) = XEssai;
X(3,:) = 0.1*ones(1,n);
X;
U = zeros(2,n);
U(1,:) = UEssai;
raid = 10000;
r = 0.1;

HX = [];
HU = [];
HV = [];

% poids
I = diag([100,dT^2,100,dT^2]);

for m = 1:n-2

for l = 1:iter

k = [];
K =[];

[nbl nbc] = size(X);
xN = X(:,nbc);
% VX = 2*I*xN;
% VXX = 2*I;

% VX = [2*raid*r^2*(raid*xN(3,1)^2*xN(1,1)-Tcible);0;0;0];
% VXX = diag([2*raid^2*xN(3,1)^4,0,0,0]);
 
[VX Lu VXX Luu Lux Lxu] = DeriveesL(xN,[0;0.1],Tcible);  
VX = 100*VX;
VXX = 100*VXX;
Vnouv = (raid*xN(3,1)^2*xN(1,1) -Tcible)^2;

for i = 1:nbc-1
    
    j = nbc-i;
    
    xi = X(:,j);
    ui = U(:,j);
    
    Vx = VX(:,1);
    Vxx = VXX(:,1:4);
    
%    Lx = LX(xi,ui,I,dT);
%    Lu = LU(xi,ui,I);
%    Lxx = LXX(xi,ui,I,dT);
%    Luu = LUU(xi,ui,I);
 
[Lx Lu Lxx Luu Lux Lxu] = DeriveesL(xi,ui,Tcible);  

%     fx = FX(xi,ui,dT);
%     fu = FU(xi,ui,dT);
    
[fx fu] = DeriveesF(xi,ui,dT);

    Qx = Lx + fx'*Vx;
    Qu = Lu + fu'*Vx;
    Qxx = Lxx + fx'*(Vxx+mu*eye(4))*fx;
    Quu = Luu + fu'*(Vxx+mu*eye(4))*fu;
    Qux = Lux + fu'*(Vxx+mu*eye(4))*fx;
    Qxu = Lxu + fx'*(Vxx+mu*eye(4))*fu;
    
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

for i = 1:nbc-1
    
    i;
    k(:,i);
    K(:,4*i-3:4*i);
    Xt(:,i) - X(:,i);
    Uti = U(:,i) + alpha*k(:,i) + K(:,4*i-3:4*i)*(Xt(:,i) - X(:,i)); 
    Xtii = f(Xt(:,i),Uti,dT);
    
%     if(Xtii(3,1)>0.15)
%        Xtii(3,1) = 0.15;
%        Uti(2,1) = 0;
%     else if(Xtii(3,1)<0.05)
%             Xtii(3,1) = 0.05;
%             Uti(2,1) = 0;
%         end;
%     end;

    Xt = [Xt Xtii];
    Ut = [Ut Uti];
    
    
    dx =  -(Xt(:,i) - X(:,i));
    Vx = VX(:,i);
    Vxx = VXX(:,4*i-3:4*i);
    dx'*Vxx*dx;
    Vx'*dx;
    Vnouv = Vnouv + Vx'*dx + 1/2* dx'*Vxx*dx;
    
    
end;

X = Xt;
U = Ut;
HV = [HV ; Vnouv];


end;

HX = [HX Xt(:,1)];
HU = [HU Ut(:,1)];

X = Xt(:,2:nbc);
U = Ut(:,2:nbc-1);

end;
% 
% HX = X;
% HU = U;

[nbl nbc] = size(HX);

T = 0:dT:(nbc-1)*dT;
% plot(T,Xt(3,:)+Xt(1,:),'r',T,Xt(3,:),'b',T,Xt(1,:),'k');
% legend('qB','qA','Theta');

Couple = raid*HX(3,:).^2.*HX(1,:);
plot(T,Couple,'r',T,HX(3,:),'b',T,HX(1,:),'g');
legend('couple','r','theta');