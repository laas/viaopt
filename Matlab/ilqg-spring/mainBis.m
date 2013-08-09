n = 15;
dT = 0.001;
Cible = 0.3;
%Tcible = 20;
iter = 15;
mu = 5e-2;
alpha = 0.1;
fenetre = 8;
qBcible = 0.5;
Kp = 50;

X = zeros(4,n);
X(3,:) = -0*Cible*ones(1,n);
U = zeros(1,n);
raid = 10000;
r = 0.1;

HX = [];
HU = [];
HqB = [];

% poids
I = diag([100,dT^2,100,dT^2]);
V = 0;

XReel = [X(:,1)];

for m = 1:350

    %Tcible = 20 * sin(2*pi*dT*m*fenetre);
    if(abs(X(1,1)+X(3,1) - qBcible)<0.01*qBcible)
        Kp = 1;
    end;
    Tcible = Kp * (X(1,1)+X(3,1) - qBcible) + 2*(Kp)^(1/2) * (X(2,1)+X(4,1));

for l = 1:iter
 
k = [];
K =[];

xN = X(:,n);
% VX = 2*I*xN;
% VXX = 2*I;

VX = 10000*[2*raid*r^2*(raid*r^2*xN(1,1)-Tcible);0;0;0];
VXX = 10000 *diag([2*raid^2*r^4,0,0,0]);
Vnouv = (raid*r^2*xN(1,1) -Tcible)^2;

for i = n-1:-1:1
    
    xi = X(:,i);
    ui = U(:,i);
    
    Vx = VX(:,1);
    Vxx = VXX(:,1:4);
    
%     Lx = LX(xi,ui,I,dT);
%    Lu = LU(xi,ui,I);
%    Lxx = LXX(xi,ui,I,dT);
%    Luu = LUU(xi,ui,I);
 
[Lx Lu Lxx Luu Lux Lxu] = DeriveesL(xi,ui,Tcible);   

    fx = FX(xi,ui,dT);
    fu = FU(xi,ui,dT);
    
    Qx = Lx + fx'*Vx;
    Qu = Lu + fu'*Vx;
    Qxx = Lxx + fx'*(Vxx+mu*eye(4))*fx;
    Quu = Luu + fu'*(Vxx+mu*eye(4))*fu;
    Qux = fu'*(Vxx+mu*eye(4))*fx;
    Qxu = fx'*(Vxx+mu*eye(4))*fu;
    
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
    Xtii = f(Xt(:,i),Uti,dT);
    Xt = [Xt Xtii];
    Ut = [Ut Uti];
    
    dx =  -(Xt(:,i) - X(:,i));
    Vx = VX(:,i);
    Vxx = VXX(:,4*i-3:4*i);
    Vnouv = Vnouv + Vx'*dx + 1/2* dx'*Vxx*dx;
    
end;

Vnouv;

% if (l==1)
%     Vanc = Vnouv;
% end;
%   
% if (abs(Vanc)>abs(Vnouv))
%     Vanc = Vnouv;
% end;

X = Xt;
U = Ut;

end;

Bruit = eye(4)+.1*2*diag([rand-0.5,rand-0.5,rand-0.5,rand-0.5]);
[nblr nbcr] = size(XReel);
X0 = [XReel(:,nbcr)]; 
HU = [HU Ut(:,1:fenetre)];
XR = Xreel(X0,U(:,1:fenetre),dT,fenetre);
U = zeros(1,n); %[Ut(:,fenetre+1:n-1) Ut(1,n-1)*ones(1,fenetre+1)];
X = Xcoherent(Bruit*XR(1:4,fenetre),U,dT);
 
XReel = [XReel XR];
% [nXr pXr] = size(XReel);
% 
% qB = XReel(3,pXr) + XReel(1,pXr);
% dqB = XReel(4,pXr) + XReel(2,pXr);
% qBstock = qAstock(1,:) + XReel(1,pXr-fenetre +1:pXr);
% HqB = [HqB  qBstock];

end;

HX = XReel;
[nbl nbc] = size(HX);

T = 0:dT:(nbc-1)*dT;
% plot(T,Xt(3,:)+Xt(1,:),'r',T,Xt(3,:),'b',T,Xt(1,:),'k');
% legend('qB','qA','Theta');

Couple = 10000*0.1^2*HX(1,:);
%plot(T,Couple,'r',T,HX(3,:),'b',T,HX(1,:),'g');
%hold on;
plot(T,HX(3,:)+HX(1,:),'k');
% legend('couple','qA','theta');
% title('Bruit de 10% - Stiffness fixe');