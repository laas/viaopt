function    Lxx = LXX(xi,ui,I,dT)

% Lxx = 2*I;

Lxx = 2*[10000 0 10000 0;0 dT^2 0 0;10000 0 10000 0;0 0 0 dT^2];