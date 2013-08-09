dT = 1;
S = 1*[100 0 0 0 0 1000 0 0 0 0 10000 0 0 0 0 100]';
R = 1;
[T,Psol] = ode45(@LQR,[1.2 0],S);
[n,p] = size(Psol);
TT = T(n:-1:1);


P = zeros(n,16);
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

