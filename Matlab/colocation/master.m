function T = master(T,StateInit,StateObj,alpha0,TimeSliding,NbWindows,n,m,Duration)

% Vector steps of time
 TimePlan = VectTimePlan(n);

% First initialisation for alpha vector
 alphaOpt = AlphaOptimal(alpha0,StateInit,n,StateObj,0,TimePlan);

% Change size of optimal alpha vector
 n=m;
 alpha0 = AlphaChangeScale(alphaOpt,n);

% Vector steps of time
 TimePlan = VectTimePlan(n);
 
% Optimization 

for j = 0:NbWindows
    
    % Optimal Solution for the current window
    alphaOpt = AlphaOptimal(alpha0,StateInit,n,StateObj,T,TimePlan);
 
    % Plot solutions of the current time window
    StateInit = PlotPartial(alphaOpt,StateInit,TimeSliding,TimePlan,T);
    
    % Slide the vector of acceleration (alpha)
    alphaO = AlphaSlided(alphaOpt,TimeSliding);

    T = T+TimeSliding*5/(12*n);
end;

axis 

 PlotAfterControl(StateInit,T,Duration);
% PlotAfterControlTry(StateInit,T,Duration,alpha0,TimePlan);

legend('qB','qA','Theta');

xlabel('Time (s)');
ylabel('Angular Position (Rd)');