function L = Lx(xi,ui,dT)

I =diag([10,dT^2,1,dT^2]);

L = 2*I*xi;
