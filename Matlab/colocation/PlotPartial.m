function StateInit = PlotPartial(alphaOpt,StateInit,TimeSliding,TimePlan,TimeStart)

T = TimeStart;
[n,p]= size(TimePlan);
SI = [0 0 0 0];

% With real system
for i = 1 :  TimeSliding
    
   dT = TimePlan(i);
  
    
   options = odeset('RelTol',1e-4,'AbsTol',[1e-5 1e-5 1e-5 1e-5]);
   [Tinit,Yinit] = ode45(@(t,q)ThetaNoPtrb(t,q,alphaOpt(i)),[T T+dT],StateInit,options);
   [k,p] = size (Yinit);
   StateInit = Yinit(k,:);   
   
   if (i == TimeSliding)
       SI = StateInit;
   end;
  
   T = T+dT;
   plot(T,(StateInit(3)-StateInit(1)),'r*',T,StateInit(3),'b+',T,StateInit(1),'g');
   hold on;

        
end;

StateInit = SI;

% % % With linearised system
% % masse volumique : alu
% rho = 2700;
% % raideur du ressort
% k = 10000;
% 
% % longueurs caract�ristiques syst�me
% L = 0.2;
% l = 0.1;
% S = 0.05^2;
% 
% % Coeff inertie
% M = rho* L^3 * S/3;
% 
% % Coeff equa diff normalisee
% delta = sqrt(k*l^2/M);
% 
% for i = 1 : TimeSliding
%     dT = TimePlan(i);
%     T = T +dT;
%     Theta = (StateInit(1)-alphaOpt(i)/delta^2) + alphaOpt(i)/delta^2;
%     ThetaPrim = StateInit(2);
%     Qa = alphaOpt(i)*(dT)^2/2 + StateInit(3) + StateInit(4)*dT;
%     QaPrim = alphaOpt(i)*dT + StateInit(4);
% 
%     StateInit = [Theta;ThetaPrim;Qa;QaPrim];
%     plot(T,(StateInit(3)-StateInit(1)),'r*',T,StateInit(3),'b+',T,StateInit(1),'g');
%     hold on;
% end;


