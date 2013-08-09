n = 100;
dT = 0.001;
Cible = 0.3;
Tcible = 20;
iter = 1;
mu = 1;
alpha = 1;

X = zeros(4,n);
% X(1:2,:) = HXEssai;
X(3,:) = 0.1*ones(1,n);
U = zeros(2,n);
% U(1,:) = HUEssai;
raid = 10000;
r = 0.1;

HX = [];
HU = [];

% poids
I = diag([100,dT^2,100,dT^2]);

for m = 1:2
m
for l = 1:iter
l
k = [];
K =[];

xN = X(:,n+1-m);
% VX = 2*I*xN;
% VXX = 2*I;

% VX = [2*raid*r^2*(raid*xN(3,1)^2*xN(1,1)-Tcible);0;0;0];
% VXX = diag([2*raid^2*xN(3,1)^4,0,0,0]);
 
[VX Lu VXX Luu Lux Lxu] = DeriveesL(xN,[0;0.1],Tcible);  

for i = 1:n-m
    
    j = n-m-i+1
    
   
    
end;

Xt = X(:,1);
Ut = [];

for i = 1:n-m
   
    
end;

end;
end;

