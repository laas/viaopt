clear all;

M =80;
dT = 0.01;
X = Solutio(M,dT);
U(1,1:M) = -5.5e3*ones(1,M);
U(2,1:M) = 0.1*ones(1,M);
raideur =100000;
Tcible = 100;
Poids = 100;
mu = 0;
alpha = 0.5;

HH =[];
J = 10;

for l=0:M-2

    N =M-l;
    H_X = [];
    V = 0;
    
for j = 0:J 



Un = U(2,N-1);
X1n = X(1,N);

Vx = zeros(2,1);
Vx(1,1) =Poids*2*raideur*Un^2*(raideur*Un^2*X1n-Tcible);



Vxx = zeros(2,2);
Vxx(1,1) = Poids * 2 * raideur^2 * Un^4;
% Vxx(3,1) = Poids * 4*raideur*X3*(2*raideur*X3^2*X1n-Tcible);
% Vxx(1,3) = Vxx(3,1);
% Vxx(3,3) = Poids * 4*raideur*X1n*(raideur*X3^2*X1n-Tcible) + 8*raideur^2*X3^2*X1n^2;


k =[];
K = [];

Vx = zeros(2,1);
Vx(1,1) =Poids*2*raideur*Un^2*(raideur*Un^2*X1n-Tcible);



Vxx = zeros(2,2);
Vxx(1,1) = Poids * 2 * raideur^2 * Un^4;

for i = N-1:-1:1
    mu = 0;
    
    Un = U(2,N-1);
    
   x = X(:,i);
   u = U(:,i);
   
   Qx = lx(u,x) + fx(x,u,dT)'*Vx(:,N-i);
   Qu = lu_(x,u) + fu(x,u,dT)'*Vx(:,N-i);
   Qxx = lxx(u,x) + fx(x,u,dT)'*(Vxx(:,2*(N-i)-1:2*(N-i)))*fx(x,u,dT);
   Quu = luu(x,u) + fu(x,u,dT)'*(Vxx(:,2*(N-i)-1:2*(N-i))+mu*eye(2))*fu(x,u,dT);
   Qux = lux(u,x)' + fu(x,u,dT)'*(Vxx(:,2*(N-i)-1:2*(N-i))+mu*eye(2))*fx(x,u,dT);
   Qxu = lux(u,x) + fx(x,u,dT)'*Vxx(:,2*(N-i)-1:2*(N-i))*fu(x,u,dT);
     
   eigen = eig(Quu);
   l1 = eigen(1,1);
   l2 = eigen(2,1);
   

   
   Vx = [(Qx-(Qu'*inv(Quu)*Qux)') Vx];
   Vxx = [(Qxx - Qxu*inv(Quu)*Qux) Vxx];

   k = [-inv(Quu)*Qu k];
   K = [-inv(Quu)*Qux K];
   
%    Vx = Qx-K(:,1:4)'*Quu*k(:,1) +K(:,1:4)'*Qu+Qux'*k(:,1);
%    Vxx = Qxx-K(:,1:4)'*Quu*K(:,1:4) +K(:,1:4)'*Qux+Qux'*K(:,1:4);
   
end;

Xopt = X(:,1);
Uopt = [];

for i=1:N-1
    Unew = (U(:,i) + alpha*k(:,i) + K(:,2*i-1:2*i)*(Xopt(:,i)-X(:,i)));
    
    if (Unew(2,1)>=0.15)
        Unew(2,1) = 0.15;
    else if (Unew(2,1) < 0.05)
            Unew(2,1) = 0.05;
        end;
    end;
    
    Uopt =[Uopt Unew];
    Xtilde = f(Xopt(:,i),Uopt(:,i),dT);
    Xopt = [Xopt Xtilde];
    
%     dX = X(:,i)-Xtilde;
%     size(dX);
%     size(Vx(:,i));
%     size(Vxx(:,2*i-1:2*i));
%     V = V+Vx(:,i)'*(dX) + 1/2*dX'*Vxx(:,2*i-1:2*i)*dX;
end;

% V = V + 10*(raideur*Uopt(2,N-1)^2*Xopt(1,N)-Tcible)^2

U = Uopt;
X = Xopt;
H_X = [H_X;X];



% X = [Xopt(:,2:N)];
% U = [Uopt(:,2:N-1)];
% j;
% cout = l(X,U,dT);
% Couple = raideur*U(2,1)^2*sin(X(1,1));
% 
% plot(j*dT,Couple,'g+',j*dT,X(1,1),'b',j*dT,U(2,1),'r');
% X(1,1);
% hold on;

end;

HH = [HH ; H_X(2*J-1,1)];

X = X(:,2:N);
U = Uopt(:,2:N-1);

end;

HH;
Couple = raideur*0.1*0.1*HH;
[N,P] = size(Couple);
T = 0:dT:(N-1)*dT;
plot(T,Couple);

