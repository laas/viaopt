function alphaOpt = AlphaOptimal(alpha0,StateInit,n,StateObj,TimeStart,TimePlan)

options = optimset('TolFun',1e-20,'Algorithm','Levenberg-Marquardt');
alphaOpt = lsqnonlin(@(alpha)CostLinTpsVar(alpha,StateInit,n,StateObj,TimeStart,TimePlan),alpha0,[],[],options);
 