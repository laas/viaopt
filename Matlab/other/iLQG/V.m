function V_retour=V(x,u)

% raideur du ressort
k = 100000;

% torque cible
Torque_cible = 100;

    V_retour = 10*u(1,1)^2 + (u(2,1)-0.1)^2 + 10*(k*u(2,1)^2*x(1,1)-Torque_cible)^2;

    