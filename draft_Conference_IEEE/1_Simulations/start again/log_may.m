function constraints = log_may(constraints, B1, b1, func, c );
    constraints = [constraints, [sum(B1)==1],... 
              implies(B1(1),[ func >=c,     b1 == 1]);
              implies(B1(2),[ func <=c,     b1 == 0]) ];
end

