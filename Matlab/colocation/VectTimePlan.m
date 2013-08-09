function T = VectTimePlan(n)

T = zeros(n,1);

for i = 1 : n
    
    if (i<=6*n/10)
        T(i) = 5/(12*n);
        
    else if(i<=8*n/10)
            T(i) = 5/(4*n);
            
        else 
            T(i) = 5/(2*n);
        end;
        
    end;
end;

