function [qA0 dqA0 qA] = Acc(qA0,dqA0,U,fenetre,dT)

qA = [];

for i = 1 : fenetre
    qA0 = qA0 + dT^2*U(1,i)/2 + dqA0*dT;
    dqA0 = dqA0 + dT*U(1,i);
    qA = [qA [qA0;dqA0]];
end;
    