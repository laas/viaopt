function V=Vx(x,u,i)

N = 2;

if (i==N)
    V = zeros(6,1);
else
    V = Qx(x,u,i+1) - Qu(x,u,i+1)*inv(Quu(x,u,i))*Qux(x,u,i);
    
end;
