function F = fcontrol(alpha,EtatInit,dT,n)

% donne les valeurs au bout du pas de temps dT
EtatCib = 0.4;
F = 0;

for i = 1 : n

%�tat au bout de dT � partir de l'�tat initial PREVU (donc sans perturbation)
options = odeset('RelTol',1e-4,'AbsTol',[1e-5 1e-5 1e-5 1e-5]);
[Tfin,Yfin] = ode45(@(t,q)ThetaNoPtrb(t,q,alpha(i)),[0 dT],EtatInit,options);
[n,p] = size (Yfin);
EtatInit = Yfin(n,:);

% % �tat final : avec alpha en entr�e pendant le pas de temps dT
% % devient �tat initial pour prochain pas de temps -> les perturbations
% options = odeset('RelTol',1e-4,'AbsTol',[1e-5 1e-5 1e-5 1e-5]);
% [Tinit,Yinit] = ode45(@(t,q)Theta(t,q,alpha(i)),[(i-1)*dT i*dT],EtatFin,options);
% [n,p] = size (Yinit);
% EtatInit = Yinit(n,:);

%plot(i*dT,(EtatInit(3)-EtatInit(1)),'r*',i*dT,EtatInit(3),'b+',i*dT,EtatInit(1),'g');
%hold on;

% if (i>45)
%     F = F + ((EtatInit(3) - EtatInit(1) - EtatCib)/EtatCib )^2;
% end;

end;

% fonction � optimiser : position finale et acc�l�rations 
F =  ((EtatInit(3) - EtatInit(1) - EtatCib))^2 + ((EtatInit(4) - EtatInit(2))*dT)^2 + 1/50*norm(alpha*dT^2)^2;
