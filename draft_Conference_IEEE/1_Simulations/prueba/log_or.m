function constraints = log_or(constraints, bin, a , b );
    constraints = [constraints, [a <= bin, b <= bin,  -a - b + bin <= 0 ]  ];
end

% constraints = log_or(constraints, N1{k}, l_eta1{k}, dis12{k}, Dl );

% constraints = [constraints, sum(N1{k})==1, 
%               implies( N1{k}(   1), [        dis12{1} <= -Dl ,     n1{k}==0 ] );
%               implies( N1{k}(2), [  -Dl<= dis12{1} <= Dl,       n1{k}==1 ] );
%               implies( N1{k}(3), [   Dl<= dis12{1}  ,           n1{k}==0 ] ) ];  