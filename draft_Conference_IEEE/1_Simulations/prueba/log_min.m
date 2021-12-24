function constraints = log_min(constraints, bin, func, c);
M=10000;
m=-M;
eps =10e-2;
constraints = [constraints, (M-c)*bin <= M - func, (c - m+eps)*bin >= eps + c - func];    
end
 
% constraints = [constraints, [sum(B1)==1],... 
%               implies(B1(1),[ func >=c,     b1 == 1]);
%               implies(B1(2),[ func <=c,     b1 == 0]) ];