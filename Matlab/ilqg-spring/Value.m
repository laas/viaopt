function v = Value(x,u,Tcible)

raid = 10000;


v = 10*(raid*x(3,1).^2*x(1,1)-Tcible).^2 + 0.1*(u(1,1).^2 + u(2,1).^2);