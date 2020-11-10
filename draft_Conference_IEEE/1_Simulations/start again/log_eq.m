function constraints = log_eq(constraints, D1, a1, func, c);
constraints = [constraints, [D1(1) + D1(2)+D1(3)==1],... 
              implies( D1(1), [  func <=c-0.01 ,                 a1==0   ]);
              implies( D1(2), [  c - 0.01 <= func <= c + 0.01,   a1==1   ]);
              implies( D1(3), [  c + 0.01 <= func,               a1==0   ]) ];
end

% 
% constraints = [constraints, [D1{k}(1)+D1{k}(2)+D1{k}(3)==1],... 
%               implies( D1{k}(1), [  z_2-z{k} <=-0.01 ,       a1{k}==0   ]);
%               implies( D1{k}(2), [  -0.01<= z_2-z{k} <=0.01,  a1{k}==1   ]);
%               implies( D1{k}(3), [  0.01 <= z_2-z{k},        a1{k}==0   ]) ];
%           