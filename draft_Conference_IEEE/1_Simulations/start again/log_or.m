function constraints = log_or(constraints, N, n, func, c );
constraints = [constraints, sum(N) == 1, 
              implies( N(1), [        func <= -c ,      n==0 ] );
              implies( N(2), [  -c <= func <= c,        n==1 ] );
              implies( N(3), [   c <= func  ,           n==0 ] ) ];  
end

% constraints = log_or(constraints, N1{k}, l_eta1{k}, dis12{k}, Dl );

% constraints = [constraints, sum(N1{k})==1, 
%               implies( N1{k}(   1), [        dis12{1} <= -Dl ,     n1{k}==0 ] );
%               implies( N1{k}(2), [  -Dl<= dis12{1} <= Dl,       n1{k}==1 ] );
%               implies( N1{k}(3), [   Dl<= dis12{1}  ,           n1{k}==0 ] ) ];  