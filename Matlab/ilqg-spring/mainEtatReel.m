clear all; 


% --- Parameters
n = 15;
dT = 0.0001;
iter = 3;
mu = 0.005;
alpha = 0.001;
Kp = 100;
fenetre = 3;
M = 80;

% --- System : vector state...
r = 0.1; % Position initiale du ressort
X = zeros(4,n); % State vector
X(3,:) = r*ones(1,n);
U = zeros(2,n); % Command vector
raid = 10000; % Stiffness of the spring
qA = 0; % Initial Position of A
dqA = 0; % ...
qB = 0; % ...
dqB = 0; % ...
HqB = [qB]; % Saving the different positions of B during time


% --- Historical vectors 
HU = zeros(2,fenetre*M);
HV = zeros(1,M);
HVX = zeros(4,n);
HVXX = zeros(4,4*n);
XReel = zeros(6,fenetre*M+1); XReel(:,1) = [X(:,1);qA;dqA]; % Complete state
HT = zeros(1,M*fenetre+1);

% --- Goal
qBcible = 0.2;
HqBcible = zeros(1,M+2);
Tcible = - Kp * (qB - qBcible) + 2*Kp^(1/2)*dqB; % Wanted Torque 
Tcible = 20;
HT(1,1) = Tcible;

% --- Initialization 
[XEssai UEssai] = Initialisation(n,dT,Tcible,r,zeros(4,n),U(1,:));
X(1:2,:) = XEssai(1:2,:);
U(1,:) = [UEssai 0];
XReel = [X(:,1);qA;dqA];
X0 = XReel;


for m = 1:M
    
    % Definition of the profile of Tcible
     if(m*fenetre*dT>0.01)
         Tcible = 50;
     end;
      if(m*fenetre*dT>0.02)
         Tcible = 30;
     end;
%        if(m*fenetre*dT>0.03)
%          Tcible = 50;
%      end;
%         if(m*fenetre*dT>0.04)
%          Tcible = 100;
%      end;
%         if(m*fenetre*dT>0.05)
%          Tcible = 40;
%      end;
     
     V = 0;
     
for l = 1:iter
    
k = zeros(2,n-1); % open-loop term
K = zeros(2,4*n); % feedback gain term

[nbl nbc] = size(X);
xN = X(:,n);

    % --- Cost function on the end point
[hvx Lu hvxx Luu Lux Lxu] = DeriveesL(xN,[0;0],Tcible);  
HVX(:,n) = 1000*hvx;
HVXX(:,4*n-3:4*n) = 10000*hvxx;

% --- Backward Pass
for i = n-1:-1:1
    
    xi = X(:,i);
    ui = U(:,i);
    
    Vx = HVX(:,i+1);
    Vxx = HVXX(:,4*i+1:4*(i+1));
     
[Lx Lu Lxx Luu Lux Lxu] = DeriveesL(xi,ui,Tcible);
[fx fu] = DeriveesF(xi,ui,dT);

    Qx = Lx + fx'*Vx;
    Qu = Lu + fu'*Vx;
    Qxx = Lxx + fx'*(Vxx+mu*eye(4))*fx;
    Quu = Luu + fu'*(Vxx+mu*eye(4))*fu;
    Qux = Lux + fu'*(Vxx+mu*eye(4))*fx;
    Qxu = Lxu + fx'*(Vxx+mu*eye(4))*fu;
    
    
%     if(isnan(norm((Quu)))==1 || rcond(Quu)<1e-15 )
%         i = nbc -1;
%         l = iter;
%         message = 'probleme';
%         erreur = 1
%     else
   
       ki = -inv(Quu)*Qu;
       Ki = -inv(Quu)*Qux;
   
        k(:,i) =  ki;
        K(:,4*i-3:4*i) = Ki;

        Vxi = Qx - Qux'*inv(Quu)*Qu;
        Vxxi = Qxx - Qxu*inv(Quu)*Qux;
    
        HVX(:,i) = Vxi;
        HVXX(:,4*i-3:4*i) = Vxxi;
        
   % end;
    
end;

% if (erreur == 1)
% else
%     VX = HVX;
%     VXX = HVXX;


Xt = zeros(4,n); Xt(:,1) = X(:,1); % new computed trajectory
Ut = zeros(2,n); % new computed command

% --- Forward pass
for i = 1:n-1
    
    Uti = U(:,i) + alpha*k(:,i) + K(:,4*i-3:4*i)*(Xt(:,i) - X(:,i));
    Xtii = f(Xt(:,i),Uti,dT);

    Xt(:,i+1) = Xtii;
    Ut(:,i) = Uti;
    
end;


% if problem during the computation, we keep the old computed trajectory
    if (isnan(norm(Xt))== 1 || isnan(norm(Ut)) == 1)
        message = 'probleme';
        l = iter;
    else
        X = Xt;
        U = Ut;
    end;
    
 
end;


% --- Real System and Reinitialization of the vectors
X0 = [X(:,1);qA;dqA]; % Start point for the "real" system
HU(:,(M-1)*fenetre+1:M*fenetre) = Ut(:,1:fenetre); % History of the command
XR = Xreel(X0,U(:,1:fenetre),dT,fenetre,m);
X0 = XR(:,fenetre); 
X = Xcoherent(XR(1:4,fenetre),zeros(2,n),dT);
XReel(:,(m-1)*fenetre+2:m*fenetre+1) = XR;

qA = XR(5,fenetre);
dqA = XR(6,fenetre);
qB = qA + XR(1,fenetre);
dqB = dqA + XR(2,fenetre);
r = XR(3,fenetre);


[XEssai UEssai] = Initialisation(n,dT,Tcible,r,[X(1:2,:);qA*ones(1,n);dqA*ones(1,n)],zeros(1,n));
X = zeros(4,n);
X(3,:) = r*ones(1,n);
X(1:2,:) = XEssai(1:2,:);
U = zeros(2,n-1);
U(1,:) = [UEssai];

HT(:,(m-1)*fenetre+2:m*fenetre+1) = Tcible*ones(1,fenetre); % History of the torque
end;


% --- Display

T = 0:dT:(M*fenetre)*dT;
Couple = raid*XReel(3,:).^2.*XReel(1,:);
plot(T,Couple,'r',T,XReel(3,:),'b',T,XReel(1,:),'g');
legend('Couple','r','theta');

% plot(T,Xt(3,:)+Xt(1,:),'r',T,Xt(3,:),'b',T,Xt(1,:),'k');
% legend('qB','qA','Theta');
%legend('couple','r','theta');
%plot(T,(HX(5,:)+HX(1,:)),'c');
%title('Commande en couple - Boucle ferm�e (5*dT) - Initialisation avec raideur fixe');

% plot(HqB);