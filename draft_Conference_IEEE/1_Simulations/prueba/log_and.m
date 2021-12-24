function constraints = log_and(constraints, bin, a , b );
    constraints = [constraints, [bin <= a, bin <= b, a + b - bin <= 1] ];
end
