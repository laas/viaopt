function V=Vxx(x,u,i)

N = 2;

if (i==N)
    V = zeros(6,6);
else
    V = Qxx(x,u,i+1) - Qux(x,u,i+1)*inv(Quu(x,u,i))*Qux(x,u,i);
    
end;
