function F = CostLinTpsVar(alpha,EtatPcdt,n,EtatCib,Tdeb,TimePlan)

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

% fixed Goal
T = Tdeb;

% sliding Coal
% T = 0;

compteur = 1;

for i = 1 : n
   
    dT = TimePlan(i);
      
    T = T+dT;
    
% B = (EtatPcdt(1)-alpha(i)/delta^2)*sin(i*dT/delta) + (delta*EtatPcdt(2))*cos(i*dT/delta);
% A = (EtatPcdt(1)-alpha(i)/delta^2)*cos(i*dT/delta) + (delta*EtatPcdt(2))*sin(i*dT/delta);
% D = EtatPcdt(4)-alpha(i)*i*dT;
% C = EtatPcdt(3)+alpha(i)*(i*dT)^2/2-i*dT*EtatPcdt(4);

Theta = (EtatPcdt(1)-alpha(i)/delta^2)*cos(dT/delta) + alpha(i)/delta^2 + delta*EtatPcdt(2)*sin(dT/delta);
ThetaPrim = EtatPcdt(2)*cos(dT/delta)-(EtatPcdt(1)-alpha(i)/delta^2)/delta*sin(dT/delta);
Qa = alpha(i)*(dT)^2/2 + EtatPcdt(3) + EtatPcdt(4)*dT;
QaPrim = alpha(i)*dT + EtatPcdt(4);

EtatPcdt = [Theta;ThetaPrim;Qa;QaPrim];

% Comparaison avec LQR
if(Tdeb<0.1)
    F((i-1)*5+1) = 10*dT^2*alpha(i);
    F((i-1)*5+2) = ((EtatPcdt(3) - EtatCib));
    F((i-1)*5+3) = (EtatPcdt(1));
    F((i-1)*5+4) = sqrt(0.1)* dT * EtatPcdt(2);
    F((i-1)*5+5) = sqrt(0.1)* dT * EtatPcdt(4);

else
    poids = sqrt(10);
    F((i-1)*5+1) = 10*dT^2*alpha(i);
    F((i-1)*5+2) = sqrt(10)* ((EtatPcdt(3) - EtatCib));
    F((i-1)*5+3) = sqrt(10)* (EtatPcdt(1));
    F((i-1)*5+4) = sqrt(10)* dT * EtatPcdt(2);
    F((i-1)*5+5) = sqrt(10)* dT * EtatPcdt(4);
end

% F(i) = 1e-5*alpha(i)*dT^2/sqrt(n);

% if(Tdeb< 0.6)
    seuil = 0.7;
% else 
%     seuil = Tdeb +0.1;
% end;

% if (T>seuil)
%         F(n+compteur) = 1000* ((EtatPcdt(3) - EtatPcdt(1) - EtatCib));
%         F(n+compteur+1) = 100*((EtatPcdt(4)- EtatPcdt(2))*dT);
%         F(n+compteur+2,1) =  ((EtatPcdt(3) - EtatCib));
%         F(n+compteur+3,1) =  (EtatPcdt(4)*dT);
%         compteur = compteur+4;
%         
% end;


end;