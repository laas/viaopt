function alpha = AlphaSlided(alphaOpt,TimeSliding)

[n,p] = size(alphaOpt);
alpha = zeros(n);

for k = 1:n-TimeSliding
    alpha(k) = alphaOpt(k+TimeSliding);
end;
for k = n-TimeSliding+1:n
    alpha(k) = 0;
end;