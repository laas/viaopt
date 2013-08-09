function q=Quu(x,u,i)

q = luu(x,u) + fu(x,u)'*Vxx(x,u,i+1)*fu(x,u);