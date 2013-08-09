function q=Qx(x,u,i)

q = lx(x,u) + fx(x,u)'*Vx(i+1);