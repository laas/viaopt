function [LX LU LXX LUU LUX LXU] = DeriveesL(xi,ui,Tcible)

% parameters
k = 10000;
L = 0.2;
l = 0.1;
S = 0.05^2;
rho = 2700;
J = rho*S*L^2;
K = k*l*l/J;

% % --- Without stiffness
% LX = [2*k*l^2*(k*l^2*xi(1,1)-Tcible);0;0;0];
% LXX = diag([2*k^2*l^4,0,0,0]);
% 
% LU = 0*2*ui;
% LUU = 0*2;
% 
% LUX = 0;
% LXU = 0;


% --- With Stiffness
LX = 0.1*[2*k*xi(3,1)^2*(k*xi(3,1)^2*xi(1,1)-Tcible);0;4*k*xi(1,1)*xi(3,1)*(k*xi(3,1)^2*xi(1,1)-Tcible) + 0.1*( -1/(0.15-xi(3,1)) - 1/(xi(3,1)-0.05));0];
LXX = 0.1*[2*k^2*xi(3,1)^4 0 8*k^2*xi(3,1)^3*xi(1,1)-4*k*xi(3,1)*Tcible 0;0 0 0 0;8*k^2*xi(3,1)^3*xi(1,1)-4*k*xi(3,1)*Tcible 0 12*k^2*xi(3,1)^2*xi(1,1)^2-4*k*xi(1,1)*Tcible+0.1*(-1/(0.15-xi(3,1))^(2)+1/(xi(3,1)-0.05)^(2)) 0;0 0 0 0];

LU = [2*0.1*(ui(1,1));2*1e-3*(ui(2,1))];
LUU = zeros(2,2); LUU(1,1) = 2*0.1;LUU(2,2) = 2*1e-3;
LUX = zeros(2,4);
LXU = LUX';

% Fonctions barrieres
% LU(2,1) =0.1*( -1/(0.15-ui(2,1)) - 1/(ui(2,1)-0.05));
% LUU(2,2) =0.1* (-1/(0.15-ui(2,1))^(2)+1/(ui(2,1)-0.05))^(2);
    

% if(xi(3,1)<0.045)
%     LU(2,1) =10;
%     LUU(2,2) =10;
% else if (xi(3,1)>0.145)
%     LU(2,1) = -10;
%     LUU(2,2) = 10;
% else
%     LU(2,1) = 0.000;
%     LUU(2,2) = 0.000;
% end;
% end;

     
%  % --- With Stiffness (state :2)
% LX = [2*k*ui(2,1)*(k*ui(2,1)^2*xi(1,1)-Tcible);0];
% LXX = diag([4*k^2*ui(2,1)^4,0]);
% 
% LU = [0;4*k^2*ui(2,1)^3*xi(1,1)^2 - 4*k*ui(2,1)*xi(1,1)*Tcible];
% LUU = diag([0,12*k^2*ui(2,1)^2*xi(1,1)^2 - 4*k*xi(1,1)*Tcible]);
% 
% LUX = [0 0;8*k^2*ui(2,1)^3*xi(1,1) - 4*k*ui(2,1)*Tcible 0] ;
% LXU = LUX' ;