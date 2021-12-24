function constraints = log_imp(constraints, g, func, a);
M=1000;
m=-M;
constraints = [constraints, m*a <= g <= M*a,...
                    -M*(1-a) <= g - func <= -m*(1-a) ];

end


% constraints = [constraints, implies(a,   g == func),         
%                             implies(1-a, g == 0),
%                                     m <= func ,
%                                     func <= M];