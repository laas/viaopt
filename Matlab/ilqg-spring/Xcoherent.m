function X = Xcoherent(X0,U,dT)

[p n] = size(U);
X = [];

for i = 1 : n
    X0 = f(X0,U(:,i),dT);
    X = [X X0];
    
end;