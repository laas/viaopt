function L=lux(x,u)

k = 100000;
Tcible = 100;

L = [0 10*4*k*u(2,1)*(2*k*x(1,1)*u(2,1)-Tcible);0 0];