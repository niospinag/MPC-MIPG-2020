wfunction constraints = log_eq1(constraints, S1, s1, func, c);
constraints = [constraints, [S1(1) + S1(2)+S1(3) + S1(4) + S1(5)==1], 
              implies( S1(1), [ s1 == 0,                 func <= -c-0.01 ]  );
              implies( S1(2), [ s1 == 1,                      func == -c ]  );
              implies( S1(3), [ s1 == 0,   -c + 0.01 <= func <= c - 0.01 ]  );
              implies( S1(4), [ s1 == 1,                       func == c ]  );
              implies( S1(5), [ s1 == 0,                c + 0.01 <= func ]) ]  ; 


end

% 
% constraints = [constraints, [D1(1) + D1(2)+D1(3)==1],... 
%               implies( D1(1), [  func <=c-0.01 ,                 a1==0   ]);
%               implies( D1(2), [  c - 0.01 <= func <= c + 0.01,   a1==1   ]);
%               implies( D1(3), [  c + 0.01 <= func,               a1==0   ]) ];
