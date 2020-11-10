function constraints = log_imp(constraints, g, func, a);
constraints = [constraints, -1000*a <= g <= 1000*a  ;
                            -1000*(1-a) <= g - func <= 1000*(1-a) ];
end
% function constraints = log_imp(constraints, g, func, a);
% constraints = [constraints, [g == func*(a)] ];
% end

% function constraints = log_imp(constraints, Z, g, func, a);
% constraints = [constraints, [Z(1) + Z(2) == 1], ... 
%               implies( Z(1), [a == 1,    g == func]);
%               implies( Z(2), [a == 0,    g == 0   ]) ];
% end