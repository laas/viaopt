function q=Qu(x,u,i)

q = lu_(x,u) + fu(x,u)'*Vx(x,u,i+1);