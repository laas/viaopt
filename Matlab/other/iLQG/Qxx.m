function q=Qxx(x,u,i)

q = lxx(x,u) + fx(x,u)'*Vxx(x,u,i+1)*fx(x,u);