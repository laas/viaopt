function L=l(x,u,dT)


% raideur du ressort
k = 100000;

% torque cible
Torque_cible = 200;
U0 = 0.05;
[p,n] = size(u);
U2 = (u(2,:)-U0*ones(1,n));

% coút sur la commande 
L_u = norm(u(1,:))+norm(100*U2);

% coút pour le torque
%L_torque = (k*x(5,1)^2*sin(x(1,1))-Torque_cible)^2;

L = L_u; % + L_torque;