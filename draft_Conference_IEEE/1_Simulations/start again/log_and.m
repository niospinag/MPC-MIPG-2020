function constraints = log_and(constraints, y, a , b );
    constraints = [constraints, [y <= a], [y <= b], [y >= a + b - 1] ];
end

% constraints = log_and(constraints, l_delta1{k}, lr{k} , lr{k} );