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

R = 1;
A = [0 1 0 0;-delta^2 0 0 0 ; 0 0 0 1 ; 0 0 0 0];
C = [0 0 0 0;0 1 0 1;0 0 0 0;0 1 0 1]/R;
S = 1*[100 0 0 0 0 1000 0 0 0 0 10000 0 0 0 0 100]';
Q = 10*eye(4);

solve('P*A + transpose(A)*P - P*C*P+ Q',P)
    

for i=1:n
    P(i,:) = Psol(n-i+1,:);
end

q0 = [0;0;-0.2;0];
B = [0;1;0;1];
K=zeros(n,4);

plot(0,q0(3,1)+0.2);
hold on;


for i=1:n-1
    P4 = [P(i,1:4);P(i,5:8);P(i,9:12);P(i,13:16)];
    K(i,:) = -1/R*B'*P4;
    
    [Tb,Q] = ode45(@(t,q)ThetaNoPtrb(t,q,K(i,:)*q0),[TT(i),TT(i+1)],q0);
    [nn,pp] = size(Tb);
    q0 = Q(nn,:)';
    
    plot(TT(i+1),q0(3,1)+0.2-q0(1,1),'+');
    hold all;
end