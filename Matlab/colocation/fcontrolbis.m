function F = fcontrolbis(alpha,EtatPcdt,dT,n,EtatCib)

% donne les valeurs au bout du pas de temps dT
% EtatCib = 0.2;

% masse volumique : alu
rho = 2700;
% raideur du ressort
k = 10000;

% longueurs caract�ristiques syst�me
L = 0.2;
l = 0.1;
S = 0.05^2;

% Coeff inertie
M = rho* L^3 * S/3;

% Coeff equa diff normalisee
delta = sqrt(k*l^2/M);

for i = 1 : n
    
% B = (EtatPcdt(1)-alpha(i)/delta^2)*sin(i*dT/delta) + (delta*EtatPcdt(2))*cos(i*dT/delta);
% A = (EtatPcdt(1)-alpha(i)/delta^2)*cos(i*dT/delta) + (delta*EtatPcdt(2))*sin(i*dT/delta);
% D = EtatPcdt(4)-alpha(i)*i*dT;
% C = EtatPcdt(3)+alpha(i)*(i*dT)^2/2-i*dT*EtatPcdt(4);

Theta = (EtatPcdt(1)-alpha(i)/delta^2) + alpha(i)/delta^2;
ThetaPrim = EtatPcdt(2);
Qa = alpha(i)*(dT)^2/2 + EtatPcdt(3) + EtatPcdt(4)*dT;
QaPrim = alpha(i)*dT + EtatPcdt(4);

EtatPcdt = [Theta;ThetaPrim;Qa;QaPrim];

F(i) = alpha(i)*dT^2/sqrt(n);

seuil = floor(n/8);

if (i>n-seuil)
        F(i-n+seuil,1) = 10* ((EtatPcdt(3) - EtatPcdt(1) - EtatCib));
        F(i-n+2*seuil,1) = 10*((EtatPcdt(4)- EtatPcdt(2))*dT);
end;


end;

% fonction a optimiser : etat final et accelerations 
% F =  ((EtatPcdt(3) - EtatPcdt(1) - EtatCib))^2 + ((EtatPcdt(4) - EtatPcdt(2))*dT)^2; % + 1/50*norm(alpha*dT^2)^2;

% F(1) = 100*((EtatPcdt(3) - EtatPcdt(1) - EtatCib));
% F(2) = ((EtatPcdt(4) - EtatPcdt(2))*dT);
% F(3) = alpha(n)*dT^2;