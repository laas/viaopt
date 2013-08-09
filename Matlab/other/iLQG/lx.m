function L=lx(x,u)

Tcible = 100;
k = 100000;

L = [10*2*k*u(2,1)^2*(k*u(2,1)^2*x(1,1) - Tcible) ; 0];
