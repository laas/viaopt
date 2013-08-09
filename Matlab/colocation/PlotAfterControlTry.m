function Yinit = PlotAfterControlTry(StateInit,TimeStart,Duration,alpha0,TimePlan)

if(Duration>0)
    
    [n,p] = size(TimePlan);
    T = TimeStart;
    
    for i = 1 : n
    options = odeset('RelTol',1e-4,'AbsTol',[1e-5 1e-5 1e-5 1e-5]);
    [Tinit,Yinit] = ode45(@(t,q)ThetaNoPtrb(t,q,alpha0(i)),[T T+TimePlan(i)],StateInit,options);
    [k,p] = size (Yinit);
    StateInit = Yinit(k,:);             
    T = T+ TimePlan(i);
    plot(T,(StateInit(3)-StateInit(1)),'r*',T,StateInit(3),'b+',T,StateInit(1),'g');
    hold on;
   
    end;
   
end;


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
% [n,p] = size(TimePlan);
% T = TimeStart;
% 
% for i = 1 : n
%     dT = TimePlan(i);
%     T = T+dT;
%     Theta = (StateInit(1)-alpha0(i)/delta^2) + alpha0(i)/delta^2;
%     ThetaPrim = StateInit(2);
%     Qa = alpha0(i)*(dT)^2/2 + StateInit(3) + StateInit(4)*dT;
%     QaPrim = alpha0(i)*dT + StateInit(4);
% 
%     StateInit = [Theta;ThetaPrim;Qa;QaPrim];
%     plot(T,(StateInit(3)-StateInit(1)),'g*',T,StateInit(3),'g+',T,StateInit(1),'g');
%     hold on;
% end;
