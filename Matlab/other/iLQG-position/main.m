
n = 1000;
dT = 0.001;
cible = 2;
u = 0.2;
alpha = 0.1;

X = Solution(n,dT,u);
X(3,:) = X(3,:)-cible*ones(1,n);
U = u*ones(1,n);

Vx = 2*diag([10,dT^2,1,dT^2])*X(:,n);
Vxx = 2*diag([10,dT^2,1,dT^2]);

k = []; K = [];

for l = 1:1

for j = 1:n-1
    
    i = n-j;
    
    xi = X(:,i);
    ui = U(:,i);
    
    Vx = Vx(:,1);
    Vxx = Vxx(:,1:4);
    
    fxi = fx(xi,ui,dT);
    fui = fu(xi,ui,dT);
    
    Qx = Lx(xi,ui,dT) + fxi'*Vx;
    Qu = Lu(xi,ui,dT) + fui'*Vx;
    Qxx = Lxx(xi,ui,dT) + fxi'*Vxx*fxi;
    Quu = Luu(xi,ui,dT) + fui'*Vxx*fui;
    Qux = fui'*Vxx*fxi;
    Qxu = fxi'*Vxx*fui;
    
    ki = -inv(Quu)*Lu(xi,ui,dT);
    Ki = -inv(Quu)*Qux;
      
    Vxii = Lx(xi,ui,dT) + Ki'*Lu(xi,ui,dT);
    Vxxii = Qxx - Ki'*Quu*Ki; 
  
    if(i<n/5)
        Vxii = [0;0;0;0];
        Vxxii = zeros(4,4);
    end;
    
    Vx = [Vx Vxii];
    Vxx = [Vxx Vxxii];

    k = [ki k];
    K = [Ki K];
    
end;

Xnew = X(:,1);
Unew = [];

for i = 1:n-1
    
    Unewi = U(:,i) + alpha*k(:,i) + K(:,4*i-3:4*i)*(Xnew(:,i)-X(:,i));
    Unew = [Unew Unewi];
    Xnew = [Xnew f(Xnew(:,i),Unewi,dT)];
    
end;

X = Xnew;
U = Unew;

end;

T=0:dT:(n-1)*dT;
plot(T,Xnew(3,:));
Xnew(3,:);

