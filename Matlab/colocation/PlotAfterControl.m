function Yinit = PlotAfterControl(StateInit,TimeStart,Duration)

if(Duration>0)
    options = odeset('RelTol',1e-4,'AbsTol',[1e-5 1e-5 1e-5 1e-5]);
    [Tinit,Yinit] = ode45(@(t,q)ThetaNoPtrb(t,q,0),[TimeStart TimeStart+Duration],StateInit,options);

    plot(Tinit(:),(Yinit(:,3)-Yinit(:,1)),'r-',Tinit(:),Yinit(:,3),'b-',Tinit(:),Yinit(:,1),'g');
    legend('qB','qA','Theta');
    hold on;
end;