function StateInit = PlotPartial(alphaOpt,StateInit,TimeSliding,TimePlan,TimeStart)

T = TimeStart

for i = 1 : TimeSliding
    
   dT = TimePlan(i);
   T = T+dT;
    
   options = odeset('RelTol',1e-4,'AbsTol',[1e-5 1e-5 1e-5 1e-5]);
   [Tinit,Yinit] = ode45(@(t,q)ThetaNoPtrb(t,q,alphaOpt(i)),[(i-1)*dT i*dT],StateInit,options);
   [k,p] = size (Yinit);
   StateInit = Yinit(k,:);          
  
   plot(T,(StateInit(3)-StateInit(1)),'r*',T,StateInit(3),'b+',T,StateInit(1),'g');
   hold on;

        
end;





