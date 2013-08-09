function q=Qux(x,u,i)

q = lux(x,u) + fu(x,u)'*Vxx(x,u,i+1)*fx(x,u);