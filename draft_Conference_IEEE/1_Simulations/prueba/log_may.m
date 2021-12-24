function constraints = log_may(constraints, bin, func, c);
M=10000;
m=-M;
eps =10e-2;
constraints = [constraints, (c-m)*bin <= func - m, ...
                        (M-c+eps)*bin >= func - c + eps ];    
end
 
% constraints = [constraints, [sum(B1)==1],... 
%               implies(B1(1),[ func >=c,     b1 == 1]);
%               implies(B1(2),[ func <=c,     b1 == 0]) ];