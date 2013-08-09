function alpha = AlphaChangeScale(alphaOpt,m)

 alpha = zeros(m,1);
 [n,p] = size(alphaOpt);
 k=1;
 for i = 1 : m
     if(i/m<=k/n)
         alpha(i) =alphaOpt(k);
     else 
         k = k+1;
         alpha(i) =alphaOpt(k);
     end;
 end;