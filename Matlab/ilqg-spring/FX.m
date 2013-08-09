function fx = FX(xi,ui,dT);

% parameters
k = 10000;
L = 0.2;
l = 0.1;
S = 0.05^2;
rho = 2700;
J = rho*S*L^2;
K = k*l*l/J;

A1 = [0 1 0 0;-k*l^2/J 0 0 0;0 0 0 1;0 0 0 0];

A2 = [0 1;-k*l^2/J 0];

fx = eye(2)+dT*A2;