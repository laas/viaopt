
function dq = actuator(t,q)
dq = zeros(2,1);    % a column vector
dq(1) = q(2);
dq(2) = -sin(q(1))+2;

