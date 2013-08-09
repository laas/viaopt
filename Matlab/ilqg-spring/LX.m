function    Lx = LX(xi,ui,I,dT);

% Lx = 2*I*xi

cible = 0.3;

L1 = 10000*(xi(1,1)+xi(3,1)-cible);
Lx = 2*[L1;dT^2*xi(2,1);L1;dT^2*xi(4,1)];