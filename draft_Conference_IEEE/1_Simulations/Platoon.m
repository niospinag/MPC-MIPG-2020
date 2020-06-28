%UNIVERSIDAD NACIONAL DE COLOMBIA
% Multi Vehicle automated drivring
%Autor: Nestor Ospina
clear
close all
clc
%----------pc casa
%  addpath('C:\gurobi811\win64\matlab') %Gurobi

% addpath(genpath('C:\Program Files\IBM\ILOG\CPLEX_Studio_Community129\cplex\matlab\x64_win64'))%cplex
% addpath(genpath('C:\Program Files\IBM\ILOG\CPLEX_Studio_Community129\cplex\examples\src\matlab'))%cplex
% addpath(genpath('C:\Users\Personal\Desktop\potential games\YALMIP-master'))
% addpath('C:\gurobi811\win64\matlab') %Gurobi

% %---------laptop asus
% addpath(genpath('C:\gurobi901\win64\matlab'))%GUROBI
% addpath(genpath('C:\Users\nesto\OneDrive\Documentos\YALMIP-master'))%yalmip



yalmip('clear')
%% PROGRAM
% Model data
nv=1; %numero de vehiculos sin el agente no cooperativo
% MPC data
Q = 5*eye(1);   %weight matrix V
R = 1*eye(1);   %weight matrix Z
N = 5;%horizon
T = 0.1; %[s]
Ds=15;%Safety distance
Dl=20; %lateral distance
V_max=80;
A_max=50;
L=6;%number of lanes
Mmax=L-1;
mmin=-L+1;
p_max=1;
delay_time = 0.3;
% 0.3

%------------estados deseados-----------
Zd = sdpvar(1,1);%carril deseado
Vd = sdpvar(1,1);%velocidad deseada

% -------------vehiculo i---------------
v = sdpvar(ones(1,N+1),ones(1,N+1)); %velocidad del vehiculo actual
a = sdpvar(ones(1,N),ones(1,N)); %aceleracion actual del vehiculo
z = intvar(ones(1,N+1),ones(1,N+1)); %carril actual

v_2 = sdpvar(1,nv);  v_3 = sdpvar(1,nv);  v_4 = sdpvar(1,nv);  %velocidad del otro vehculo
z_2 = intvar(1,nv);  z_3 = intvar(1,nv);  z_4 = intvar(1,nv);  z_obs = intvar(1,nv);  %carril del vehiculo j

dis12 = sdpvar(ones(1,N+1),ones(1,N+1));  % distancia entre vehiculo 1 y 2
dis13 = sdpvar(ones(1,N+1),ones(1,N+1));  % distancia entre vehiculo 1 y 3
dis14 = sdpvar(ones(1,N+1),ones(1,N+1));  % distancia entre vehiculo 1 y 4
dis_obs=sdpvar(ones(1,N+1),ones(1,N+1));% distancia entre vehiculo 1 y obs


a1 = binvar(ones(1,N),ones(1,N));    a2 = binvar(ones(1,N),ones(1,N));    a3 = binvar(ones(1,N),ones(1,N));    a0 = binvar(ones(1,N),ones(1,N));
b1 = binvar(ones(1,N),ones(1,N));    b2 = binvar(ones(1,N),ones(1,N));    b3 = binvar(ones(1,N),ones(1,N));    b0 = binvar(ones(1,N),ones(1,N));
g1 = binvar(ones(1,N),ones(1,N));    g2 = binvar(ones(1,N),ones(1,N));    g3 = binvar(ones(1,N),ones(1,N));    g0 = binvar(ones(1,N),ones(1,N));
s1 = binvar(ones(1,N),ones(1,N));    s2 = binvar(ones(1,N),ones(1,N));    s3 = binvar(ones(1,N),ones(1,N));    s0 = binvar(ones(1,N),ones(1,N));
n1 = binvar(ones(1,N),ones(1,N));    n2 = binvar(ones(1,N),ones(1,N));    n3 = binvar(ones(1,N),ones(1,N));    n0 = binvar(ones(1,N),ones(1,N));

Aa1 = binvar( 1,N );                 Aa2 = binvar( 1,N );                 Aa3 = binvar( 1,N );                 Aa0 = binvar( 1,N );
Bb1 = binvar( 1,N );                 Bb2 = binvar( 1,N );                 Bb3 = binvar( 1,N );                 Bb0 = binvar( 1,N );
Gg1 = binvar( 1,N );                 Gg2 = binvar( 1,N );                 Gg3 = binvar( 1,N );                 Gg0 = binvar( 1,N );
Ss1 = binvar( 1,N );                 Ss2 = binvar( 1,N );                 Ss3 = binvar( 1,N );                 Ss0 = binvar( 1,N );
Nn1 = binvar( 1,N );                 Nn2 = binvar( 1,N );                 Nn3 = binvar( 1,N );                 Nn0 = binvar( 1,N );

D1 = binvar(3*ones(1,N),ones(1,N));  D2 = binvar(3*ones(1,N),ones(1,N));  D3 = binvar(3*ones(1,N),ones(1,N));  D0 = binvar(3*ones(1,N),ones(1,N));
B1 = binvar(2*ones(1,N),ones(1,N));  B2 = binvar(2*ones(1,N),ones(1,N));  B3 = binvar(2*ones(1,N),ones(1,N));  B0 = binvar(2*ones(1,N),ones(1,N));
G1 = binvar(3*ones(1,N),ones(1,N));  G2 = binvar(3*ones(1,N),ones(1,N));  G3 = binvar(3*ones(1,N),ones(1,N));  G0 = binvar(3*ones(1,N),ones(1,N));
S1 = binvar(5*ones(1,N),ones(1,N));  S2 = binvar(5*ones(1,N),ones(1,N));  S3 = binvar(5*ones(1,N),ones(1,N));  S0 = binvar(5*ones(1,N),ones(1,N));
N1 = binvar(3*ones(1,N),ones(1,N));  N2 = binvar(3*ones(1,N),ones(1,N));  N3 = binvar(3*ones(1,N),ones(1,N));  N0 = binvar(3*ones(1,N),ones(1,N));


p_z = sdpvar(1);


%% making the optimizer with 2 node
constraints = [];
constraints = [constraints,  diff([p_z z{1}]) == 0];
objective   = 0;
constraints = [constraints, [1 <= z{1}        <= L]];
constraints = [constraints,  1 <=    z_2      <= L];
constraints = [constraints,  1 <=    z_3      <= L];  
constraints = [constraints, [dis12{1} <= 1000]]; 
constraints = [constraints, [dis13{1} <= 1000]]; 

for k = 1:N
 objective = objective+( v{k+1}-Vd )'*Q*( v{k+1}-Vd ) + (z{k+1} - Zd)'*R*(z{k+1} - Zd); % calculate obj

  % Feasible region
    constraints = [constraints,1 <=    z{k+1}     <= L,  %tome valores posibles
                                0<=    v{k+1}   <= V_max,   %no exceda las velocidades 
                         z{k}-[p_max]<=    z{k+1}   <=z{k}+[p_max], %paso de un carril
                           -A_max<=    a{k}     <= A_max];


    constraints = [constraints, -1 <= [z{k+1} - z{k}] <= 1];
    constraints = [constraints, v{k+1} == v{k} + T*a{k}];             %velocidad futura

   
% ---------------------------------------vehiculo 2-------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, -1000  <=  dis12{k+1} <= 1000];
%     constraints = [constraints,   mmin  <= z_2-z{k+1}  <= Mmax];
    constraints = [constraints, dis12{k+1} == dis12{k} + T*(v_2-v{k})];

%.........................alpha...............................

constraints = [constraints, [D1{k}(1)+D1{k}(2)+D1{k}(3)==1],... 
              implies( D1{k}(1), [  z_2-z{k} <=-0.01 ,        a1{k}==0   ]);
              implies( D1{k}(2), [  -0.01<= z_2-z{k} <=0.01,  a1{k}==1   ]);
              implies( D1{k}(3), [  0.01 <= z_2-z{k},         a1{k}==0   ]) ];
          
%................................... Beta ....................................
constraints = [constraints, [sum(B1{k})==1],... 
              implies(B1{k}(1),[ dis12{k} >=0,     b1{k}==1]);
              implies(B1{k}(2),[ dis12{k} <=0,     b1{k}==0]) ];

          
constraints = [constraints,  Aa1(k)*( Bb1(k) *(Ds - dis12{k+1}) + (1 - Bb1(k)) * (Ds + dis12{k+1})) <= 0 ];

% %.........................Lateral distance...............................

constraints = [constraints, [sum(S1{k})==1], 
              implies( S1{k}(1), [ s1{k} == 0,        z_2-z{k} <= -1.001 ]  );
              implies( S1{k}(2), [ s1{k} == 1,        z_2-z{k} == -1 ]  );
              implies( S1{k}(3), [ s1{k} == 0,   -0.999 <= z_2-z{k} <= 0.999 ]  );
              implies( S1{k}(4), [ s1{k} == 1,         z_2-z{k} == 1 ]  );
              implies( S1{k}(5), [ s1{k} == 0,      1.001 <= z_2-z{k} ]) ]  ;   

%.........................lateral safety distance...............................
constraints = [constraints, sum(N1{k})==1, 
              implies( N1{k}(1), [        dis12{1} <= -Dl ,     n1{k}==0 ] );
              implies( N1{k}(2), [  -Dl<= dis12{1} <= Dl,       n1{k}==1 ] );
              implies( N1{k}(3), [   Dl<= dis12{1}  ,           n1{k}==0 ] ) ];   
 
          
constraints = [constraints, Ss1(k)*(Nn1(k))*(z{k+1} - z{k})==0];



% --------------------------------------- vehiculo 3 -------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds -----------------

    constraints = [constraints, -1000  <=  dis13{k+1} <= 1000];
%     constraints = [constraints,    mmin  <= z_3 - z{k+1}  <= Mmax];
    constraints = [constraints, dis13{k+1} == dis13{k} + T*(v_3-v{k})];

%.........................alpha...............................

constraints = [constraints, [D2{k}(1)+D2{k}(2)+D2{k}(3)==1],... 
              implies( D2{k}(1), [  z_3-z{k} <=-0.01 ,       a2{k}==0   ]);
              implies( D2{k}(2), [  -0.01<= z_3-z{k} <=0.01,  a2{k}==1   ]);
              implies( D2{k}(3), [  0.01 <= z_3-z{k},        a2{k}==0   ]) ];
          
%................................... Beta ....................................
constraints = [constraints, [sum(B2{k})==1],... 
              implies(B2{k}(1),[ dis13{k} >=0,     b2{k}==1]);
              implies(B2{k}(2),[ dis13{k} <=0,     b2{k}==0]) ];
          
constraints = [constraints,  Aa2(k)*( Bb2(k) *(Ds - dis13{k+1}) + (1-Bb2(k)) * (Ds + dis13{k+1})) <= 0 ];

%.............................Lateral distance..................................

constraints = [constraints, [sum(S2{k})==1], 
              implies( S2{k}(1), [ s2{k} == 0,        z_3-z{k} <= -1.1 ]  );
              implies( S2{k}(2), [ s2{k} == 1,        z_3-z{k} == -1 ]  );
              implies( S2{k}(3), [ s2{k} == 0,   -0.999 <= z_3-z{k} <= 0.999 ]  );
              implies( S2{k}(4), [ s2{k} == 1,         z_3-z{k} == 1 ]  );
              implies( S2{k}(5), [ s2{k} == 0,    1.001 <= z_3-z{k} ]) ]  ;   

%.........................lateral safety distance...............................
constraints = [constraints, sum(N2{k})==1, 
              implies( N2{k}(1), [        dis13{1} <= -Dl ,     n2{k}==0 ] );
              implies( N2{k}(2), [  -Dl<= dis13{1} <= Dl,       n2{k}==1 ] );
              implies( N2{k}(3), [   Dl<= dis13{1}  ,           n2{k}==0 ] ) ];   

constraints = [constraints, Ss2(k)*(Nn2(k))*(z{k+1}-z{k})==0];

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables

end

objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj

parameters_in = { Vd , Zd , v{1} , p_z , ...
    v_2 , z_2 , dis12{1} , Aa1 , Bb1 , Ss1 , Nn1, ...
    v_3 , z_3 , dis13{1} , Aa2 , Bb2 , Ss2 , Nn2};

solutions_out = {[a{:}], [z{:}], [v{:}], [a1{:}],  [b1{:}] ,  [s1{:}], [n1{:}],...
    [a2{:}],  [b2{:}] ,  [s2{:}], [n2{:}] };

controller2 = optimizer(constraints, objective , sdpsettings('solver','gurobi'),parameters_in,solutions_out);

%% making the optimizer with 3 node
constraints = [];
constraints = [constraints,  diff([p_z z{1}]) == 0];
objective   = 0;
constraints = [constraints, [1 <=   z{1}    <=   L   ]];
constraints = [constraints,  1 <=    z_2      <= L];
constraints = [constraints,  1 <=    z_3      <= L];  
constraints = [constraints,  1 <=    z_4      <= L];  
constraints = [constraints, [dis12{1} <= 1000]]; 
constraints = [constraints, [dis13{1} <= 1000]]; 
constraints = [constraints, [dis14{1} <= 1000]]; 

for k = 1:N
 objective = objective+( v{k+1}-Vd )'*Q*( v{k+1}-Vd ) + (z{k+1} - Zd)'*R*(z{k+1} - Zd); % calculate obj

  % Feasible region
    constraints = [constraints,1 <=    z{k+1}     <= L,  %tome valores posibles
                                0<=    v{k+1}   <= V_max,   %no exceda las velocidades 
                         z{k}-[p_max]<=    z{k+1}   <=z{k}+[p_max], %paso de un carril
                           -A_max<=    a{k}     <= A_max];

    constraints = [constraints, -1 <= [z{k+1} - z{k}] <= 1];
    constraints = [constraints, v{k+1} == v{k} + T*a{k}];             %velocidad futura

% ---------------------------------------vehiculo 2-------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, -1000  <=  dis12{k+1} <= 1000];
%     constraints = [constraints,   mmin  <= z_2-z{k+1}  <= Mmax];
    constraints = [constraints, dis12{k+1} == dis12{k} + T*(v_2-v{k})];

%.........................alpha...............................

constraints = [constraints, [D1{k}(1)+D1{k}(2)+D1{k}(3)==1],... 
              implies( D1{k}(1), [  z_2-z{k} <=-0.01 ,        a1{k}==0   ]);
              implies( D1{k}(2), [  -0.01<= z_2-z{k} <=0.01,  a1{k}==1   ]);
              implies( D1{k}(3), [  0.01 <= z_2-z{k},         a1{k}==0   ]) ];
          
%................................... Beta ....................................
constraints = [constraints, [sum(B1{k})==1],... 
              implies(B1{k}(1),[ dis12{k} >=0,     b1{k}==1]);
              implies(B1{k}(2),[ dis12{k} <=0,     b1{k}==0]) ];
          
constraints = [constraints,  Aa1(k)*( Bb1(k) *(Ds - dis12{k+1}) + (1 - Bb1(k)) * (Ds + dis12{k+1})) <= 0 ];

% %.........................Lateral distance...............................

constraints = [constraints, [sum(S1{k})==1], 
              implies( S1{k}(1), [ s1{k} == 0,        z_2-z{k} <= -1.001 ]  );
              implies( S1{k}(2), [ s1{k} == 1,        z_2-z{k} == -1 ]  );
              implies( S1{k}(3), [ s1{k} == 0,   -0.999 <= z_2-z{k} <= 0.999 ]  );
              implies( S1{k}(4), [ s1{k} == 1,         z_2-z{k} == 1 ]  );
              implies( S1{k}(5), [ s1{k} == 0,      1.001 <= z_2-z{k} ]) ]  ;   

%.........................lateral safety distance...............................

constraints = [constraints, sum(N1{k})==1, 
              implies( N1{k}(1), [        dis12{1} <= -Dl ,     n1{k}==0 ] );
              implies( N1{k}(2), [  -Dl<= dis12{1} <= Dl,       n1{k}==1 ] );
              implies( N1{k}(3), [   Dl<= dis12{1}  ,           n1{k}==0 ] ) ];   
 
constraints = [constraints, Ss1(k)*(Nn1(k))*(z{k+1} - z{k})==0];

% --------------------------------------- vehiculo 3 -------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds -----------------

    constraints = [constraints, -1000  <=  dis13{k+1} <= 1000];
%     constraints = [constraints,    mmin  <= z_3 - z{k+1}  <= Mmax];
    constraints = [constraints, dis13{k+1} == dis13{k} + T*(v_3-v{k})];

%.........................alpha...............................

constraints = [constraints, [D2{k}(1)+D2{k}(2)+D2{k}(3)==1],... 
              implies( D2{k}(1), [  z_3-z{k} <=-0.01 ,       a2{k}==0   ]);
              implies( D2{k}(2), [  -0.01<= z_3-z{k} <=0.01,  a2{k}==1   ]);
              implies( D2{k}(3), [  0.01 <= z_3-z{k},        a2{k}==0   ]) ];

%................................... Beta ....................................

constraints = [constraints, [sum(B2{k})==1],... 
              implies(B2{k}(1),[ dis13{k} >=0,     b2{k}==1]);
              implies(B2{k}(2),[ dis13{k} <=0,     b2{k}==0])];

constraints = [constraints,  Aa2(k)*( Bb2(k) *(Ds - dis13{k+1}) + (1-Bb2(k)) * (Ds + dis13{k+1})) <= 0 ];

%..............................Lateral distance...............................

constraints = [constraints, [sum(S2{k})==1], 
              implies( S2{k}(1), [ s2{k} == 0,        z_3-z{k} <= -1.1 ]  );
              implies( S2{k}(2), [ s2{k} == 1,        z_3-z{k} == -1 ]  );
              implies( S2{k}(3), [ s2{k} == 0,   -0.999 <= z_3-z{k} <= 0.999 ]  );
              implies( S2{k}(4), [ s2{k} == 1,         z_3-z{k} == 1 ]  );
              implies( S2{k}(5), [ s2{k} == 0,    1.001 <= z_3-z{k} ]) ]  ;   

%.........................lateral safety distance...............................
constraints = [constraints, sum(N2{k})==1, 
              implies( N2{k}(1), [        dis13{1} <= -Dl ,     n2{k}==0 ] );
              implies( N2{k}(2), [  -Dl<= dis13{1} <= Dl,       n2{k}==1 ] );
              implies( N2{k}(3), [   Dl<= dis13{1}  ,           n2{k}==0 ] ) ];   
 
          
constraints = [constraints, Ss2(k)*(Nn2(k))*(z{k+1}-z{k})==0];


% ---------------------------------------vehiculo 4-------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, -1000  <=  dis14{k+1} <= 1000];
%     constraints = [constraints,   mmin  <= z_4-z{k+1}  <= Mmax];
    constraints = [constraints, dis14{k+1} == dis14{k} + T*(v_4-v{k})];

%.........................alpha...............................

constraints = [constraints, [D3{k}(1)+D3{k}(2)+D3{k}(3)==1],... 
              implies( D3{k}(1), [  z_4-z{k} <=-0.01 ,        a3{k}==0   ]);
              implies( D3{k}(2), [  -0.01<= z_4-z{k} <=0.01,  a3{k}==1   ]);
              implies( D3{k}(3), [  0.01 <= z_4-z{k},         a3{k}==0   ]) ];
          
%................................... Beta ....................................
constraints = [constraints, [sum(B3{k})==1],... 
              implies(B3{k}(1),[ dis14{k} >=0,     b3{k}==1]);
              implies(B3{k}(2),[ dis14{k} <=0,     b3{k}==0])];

          
constraints = [constraints,  Aa3(k)*( Bb3(k) *(Ds - dis14{k+1}) + (1 - Bb3(k)) * (Ds + dis14{k+1})) <= 0 ];

%.........................Lateral distance...............................

constraints = [constraints, [sum(S3{k})==1], 
              implies( S3{k}(1), [ s3{k} == 0,        z_4-z{k} <= -1.001 ]  );
              implies( S3{k}(2), [ s3{k} == 1,        z_4-z{k} == -1 ]  );
              implies( S3{k}(3), [ s3{k} == 0,   -0.999 <= z_4-z{k} <= 0.999 ]  );
              implies( S3{k}(4), [ s3{k} == 1,         z_4-z{k} == 1 ]  );
              implies( S3{k}(5), [ s3{k} == 0,      1.001 <= z_4-z{k} ]) ]  ;   

%.........................lateral safety distance...............................
constraints = [constraints, sum(N3{k})==1, 
              implies( N3{k}(1), [        dis14{1} <= -Dl ,     n3{k}==0 ] );
              implies( N3{k}(2), [  -Dl<= dis14{1} <= Dl,       n3{k}==1 ] );
              implies( N3{k}(3), [   Dl<= dis14{1}  ,           n3{k}==0 ] ) ];   
 
          
constraints = [constraints, Ss3(k)*(Nn3(k))*(z{k+1} - z{k})==0];




    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
    
end

objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj

parameters_in = { Vd, Zd, v{1}, p_z, ...
    v_2, z_2, dis12{1}, Aa1, Bb1, Ss1, Nn1, ...
    v_3, z_3, dis13{1}, Aa2, Bb2, Ss2, Nn2, ...
    v_4, z_4, dis14{1}, Aa3, Bb3, Ss3, Nn3};

solutions_out = {[a{:}], [z{:}], [v{:}], [a1{:}],  [b1{:}] ,  [s1{:}], [n1{:}],...
    [a2{:}],  [b2{:}] ,  [s2{:}], [n2{:}],...
    [a3{:}],  [b3{:}] ,  [s3{:}], [n3{:}]};

controller3 = optimizer(constraints, objective , sdpsettings('solver','gurobi'),parameters_in,solutions_out);

%% making the optimizer with 3 node and obstacle
constraints = [];
objective   = 0;
constraints = [constraints,  diff([p_z z{1}]) == 0];
constraints = [constraints, [1 <=   z{1}    <=   L   ]];
constraints = [constraints,  1 <=    z_2      <= L];
constraints = [constraints,  1 <=    z_3      <= L];  
constraints = [constraints,  1 <=    z_4      <= L];  
constraints = [constraints,  1 <=    z_obs      <= L];  
constraints = [constraints, [dis12{1} <= 1000]]; 
constraints = [constraints, [dis13{1} <= 1000]]; 
constraints = [constraints, [dis14{1} <= 1000]]; 
constraints = [constraints, [dis_obs{1} <= 1000]];

for k = 1:N
 objective = objective+( v{k+1}-Vd )'*Q*( v{k+1}-Vd ) + (z{k+1} - Zd)'*R*(z{k+1} - Zd); % calculate obj

  % Feasible region
    constraints = [constraints,1 <=    z{k+1}     <= L,  %tome valores posibles
                                0<=    v{k+1}   <= V_max,   %no exceda las velocidades 
                         z{k}-[p_max]<=    z{k+1}   <=z{k}+[p_max], %paso de un carril
                           -A_max<=    a{k}     <= A_max];


    constraints = [constraints, -1 <= [z{k+1} - z{k}] <= 1];
    constraints = [constraints, v{k+1} == v{k} + T*a{k}];             %velocidad futura

   
% ---------------------------------------vehiculo 2-------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, -1000  <=  dis12{k+1} <= 1000];
%     constraints = [constraints,   mmin  <= z_2-z{k+1}  <= Mmax];
    constraints = [constraints, dis12{k+1} == dis12{k} + T*(v_2-v{k})];

%.........................alpha...............................

constraints = [constraints, [D1{k}(1)+D1{k}(2)+D1{k}(3)==1],... 
              implies( D1{k}(1), [  z_2-z{k} <=-0.01 ,        a1{k}==0   ]);
              implies( D1{k}(2), [  -0.01<= z_2-z{k} <=0.01,  a1{k}==1   ]);
              implies( D1{k}(3), [  0.01 <= z_2-z{k},         a1{k}==0   ]) ];

%................................... Beta ....................................
constraints = [constraints, [sum(B1{k})==1],... 
              implies(B1{k}(1),[ dis12{k} >=0,     b1{k}==1]);
              implies(B1{k}(2),[ dis12{k} <=0,     b1{k}==0]) ];

constraints = [constraints,  Aa1(k)*( Bb1(k) *(Ds - dis12{k+1}) + (1 - Bb1(k)) * (Ds + dis12{k+1})) <= 0 ];

% %.........................Lateral distance...............................

constraints = [constraints, [sum(S1{k})==1], 
              implies( S1{k}(1), [ s1{k} == 0,        z_2-z{k} <= -1.001 ]  );
              implies( S1{k}(2), [ s1{k} == 1,        z_2-z{k} == -1 ]  );
              implies( S1{k}(3), [ s1{k} == 0,   -0.999 <= z_2-z{k} <= 0.999 ]  );
              implies( S1{k}(4), [ s1{k} == 1,         z_2-z{k} == 1 ]  );
              implies( S1{k}(5), [ s1{k} == 0,      1.001 <= z_2-z{k} ]) ]  ;   

%.........................lateral safety distance...............................
constraints = [constraints, sum(N1{k})==1, 
              implies( N1{k}(1), [        dis12{1} <= -Dl ,     n1{k}==0 ] );
              implies( N1{k}(2), [  -Dl<= dis12{1} <= Dl,       n1{k}==1 ] );
              implies( N1{k}(3), [   Dl<= dis12{1}  ,           n1{k}==0 ] ) ];   
 
          
constraints = [constraints, Ss1(k)*(Nn1(k))*(z{k+1} - z{k})==0];



% --------------------------------------- vehiculo 3 -------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds -----------------

    constraints = [constraints, -1000  <=  dis13{k+1} <= 1000];
%     constraints = [constraints,    mmin  <= z_3 - z{k+1}  <= Mmax];
    constraints = [constraints, dis13{k+1} == dis13{k} + T*(v_3-v{k})];

%.........................alpha...............................

constraints = [constraints, [D2{k}(1)+D2{k}(2)+D2{k}(3)==1],... 
              implies( D2{k}(1), [  z_3-z{k} <=-0.01 ,       a2{k}==0   ]);
              implies( D2{k}(2), [  -0.01<= z_3-z{k} <=0.01,  a2{k}==1   ]);
              implies( D2{k}(3), [  0.01 <= z_3-z{k},        a2{k}==0   ]) ];
          
%................................... Beta ....................................
constraints = [constraints, [sum(B2{k})==1],... 
              implies(B2{k}(1),[ dis13{k} >=0,     b2{k}==1]);
              implies(B2{k}(2),[ dis13{k} <=0,     b2{k}==0]) ];
          
constraints = [constraints,  Aa2(k)*( Bb2(k) *(Ds - dis13{k+1}) + (1-Bb2(k)) * (Ds + dis13{k+1})) <= 0 ];

% %.........................Lateral distance...............................

constraints = [constraints, [sum(S2{k})==1], 
              implies( S2{k}(1), [ s2{k} == 0,        z_3-z{k} <= -1.1 ]  );
              implies( S2{k}(2), [ s2{k} == 1,        z_3-z{k} == -1 ]  );
              implies( S2{k}(3), [ s2{k} == 0,   -0.999 <= z_3-z{k} <= 0.999 ]  );
              implies( S2{k}(4), [ s2{k} == 1,         z_3-z{k} == 1 ]  );
              implies( S2{k}(5), [ s2{k} == 0,    1.001 <= z_3-z{k} ]) ]  ;   

%.........................lateral safety distance...............................
constraints = [constraints, sum(N2{k})==1, 
              implies( N2{k}(1), [        dis13{1} <= -Dl ,     n2{k}==0 ] );
              implies( N2{k}(2), [  -Dl<= dis13{1} <= Dl,       n2{k}==1 ] );
              implies( N2{k}(3), [   Dl<= dis13{1}  ,           n2{k}==0 ] ) ];   
 
          
constraints = [constraints, Ss2(k)*(Nn2(k))*(z{k+1}-z{k})==0];


% ---------------------------------------vehiculo 4-------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, -1000  <=  dis14{k+1} <= 1000];
%     constraints = [constraints,   mmin  <= z_4-z{k+1}  <= Mmax];
    constraints = [constraints, dis14{k+1} == dis14{k} + T*(v_4-v{k})];

%.........................alpha...............................

constraints = [constraints, [D3{k}(1)+D3{k}(2)+D3{k}(3)==1],... 
              implies( D3{k}(1), [  z_4-z{k} <=-0.01 ,        a3{k}==0   ]);
              implies( D3{k}(2), [  -0.01<= z_4-z{k} <=0.01,  a3{k}==1   ]);
              implies( D3{k}(3), [  0.01 <= z_4-z{k},         a3{k}==0   ]) ];
          
%................................... Beta ....................................
constraints = [constraints, [sum(B3{k})==1],... 
              implies(B3{k}(1),[ dis14{k} >=0,     b3{k}==1]);
              implies(B3{k}(2),[ dis14{k} <=0,     b3{k}==0])];

          
constraints = [constraints,  Aa3(k)*( Bb3(k) *(Ds - dis14{k+1}) + (1 - Bb3(k)) * (Ds + dis14{k+1})) <= 0 ];

%.........................Lateral distance...............................

constraints = [constraints, [sum(S3{k})==1], 
              implies( S3{k}(1), [ s3{k} == 0,        z_4-z{k} <= -1.001 ]  );
              implies( S3{k}(2), [ s3{k} == 1,        z_4-z{k} == -1 ]  );
              implies( S3{k}(3), [ s3{k} == 0,   -0.999 <= z_4-z{k} <= 0.999 ]  );
              implies( S3{k}(4), [ s3{k} == 1,         z_4-z{k} == 1 ]  );
              implies( S3{k}(5), [ s3{k} == 0,      1.001 <= z_4-z{k} ]) ]  ;   

%.........................lateral safety distance...............................
constraints = [constraints, sum(N3{k})==1, 
              implies( N3{k}(1), [        dis14{1} <= -Dl ,     n3{k}==0 ] );
              implies( N3{k}(2), [  -Dl<= dis14{1} <= Dl,       n3{k}==1 ] );
              implies( N3{k}(3), [   Dl<= dis14{1}  ,           n3{k}==0 ] ) ];   
 
          
constraints = [constraints, Ss3(k)*(Nn3(k))*(z{k+1} - z{k})==0];

% % --------------------------------------- OBSTACLE -------------------------------    
% % ------------------ si dz=0  -------------------->>>    dij >= Ds -----------------
% 
%     constraints = [constraints, -1000  <=  dis_obs{k+1} <= 1000];
%     constraints = [constraints,    mmin  <= z_obs-z{k+1}  <= Mmax];
%     constraints = [constraints, dis_obs{k+1} == dis_obs{k} + T*(-v{k})];
% 
% %.........................alpha...............................
% 
% constraints = [constraints, [D0{k}(1)+D0{k}(2)+D0{k}(3)==1],... 
%               implies( D0{k}(1), [  z_obs-z{k} <=-0.01 ,        a0{k}==0    ]);
%               implies( D0{k}(2), [  -0.01<= z_obs-z{k} <=0.01,  a0{k}==1   ]);
%               implies( D0{k}(3), [  0.01 <= z_obs-z{k},         a0{k}==0    ]) ];
% 
% %................................... Beta ....................................
% constraints = [constraints, [sum(B2{k})==1],... 
%               implies(B2{k}(1),[ dis_obs{k} >=0,     b2{k}==1]);
%               implies(B2{k}(2),[ dis_obs{k} <=0,     b2{k}==0]) ];
% 
% % constraints = [constraints,  Aa0(k)*(Ds - dis_obs{k+1}) <= 0 ];
% 
% % %.........................Lateral distance...............................
% 
% constraints = [constraints, [sum(S0{k})==1], 
%               implies( S0{k}(1), [ s0{k} == 0,             z_obs - z{k+1} <= -1.1 ]  );
%               implies( S0{k}(2), [ s0{k} == 1,             z_obs - z{k+1} == -1 ]  );
%               implies( S0{k}(3), [ s0{k} == 0,   -0.999 <= z_obs - z{k+1} <= 0.999 ]  );
%               implies( S0{k}(4), [ s0{k} == 1,               z_obs-z{k+1} == 1 ]);
%               implies( S0{k}(5), [ s0{k} == 0,         1.001 <= z_obs - z{k+1} ]) ]  ;   
% 
% %.........................lateral safety distance...............................
% constraints = [constraints, sum(N0{k})==1, 
%               implies( N0{k}(1), [        dis_obs{1} <= -Dl ,     n0{k}==0 ] );
%               implies( N0{k}(2), [  -Dl<= dis_obs{1} <= Dl,       n0{k}==1 ] );
%               implies( N0{k}(3), [   Dl<= dis_obs{1}  ,           n0{k}==0 ] ) ];   
%  
% 
% % constraints = [constraints, Ss0(k)*(Nn0(k))*(z{k+1}-z_obs)==1];


    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables

end

objective = objective + (v{N+1} - Vd)'*Q*(v{N+1} - Vd) + (z{N+1} - Zd)'*R*(z{N+1} - Zd); % calculate obj
  
% solver definition  2 node

parameters_in = { Vd , Zd , v{1} , p_z , ...
    v_2 , z_2 , dis12{1} , Aa1 , Bb1 , Ss1 , Nn1, ...
    v_3 , z_3 , dis13{1} , Aa2 , Bb2 , Ss2 , Nn2, ...
    v_4 , z_4 , dis14{1} , Aa3 , Bb3 , Ss3 , Nn3, ...
        z_obs , dis_obs{1},           Ss0 , Nn0};

solutions_out = {[a{:}], [z{:}], [v{:}], [a1{:}],  [b1{:}], [s1{:}], [n1{:}], ...
                                          [a2{:}], [b2{:}], [s2{:}], [n2{:}], ...
                                          [a3{:}], [b3{:}], [s3{:}], [n3{:}], ...
                                                            [s0{:}], [n0{:}]};

controller4 = optimizer(constraints, objective , sdpsettings('solver','gurobi'),parameters_in,solutions_out);




%% Building variables

%define las condiciones iniciales que deben tener las variables logicas
%.....................vehiculo 1..........................
alogic1_1=[zeros(1,N)]; blogic1_1=[ones(1,N)];  G1logic_1=[ones(1,N)];  S1logic_1=[ones(1,N)];  N1logic_1=[ones(1,N)];  
alogic2_1=[zeros(1,N)]; blogic2_1=[ones(1,N)];  G2logic_1=[ones(1,N)];  S2logic_1=[ones(1,N)];  N2logic_1=[ones(1,N)];
alogic3_1=[zeros(1,N)]; blogic3_1=[ones(1,N)];  G3logic_1=[ones(1,N)];  S3logic_1=[ones(1,N)];  N3logic_1=[ones(1,N)];
alogic0_1=[zeros(1,N)]; blogic0_1=[ones(1,N)];  G0logic_1=[ones(1,N)];  S0logic_1=[ones(1,N)];  N0logic_1=[ones(1,N)];

%..................... vehiculo 2 ........................
alogic1_2=[zeros(1,N)]; blogic1_2=[ones(1,N)];  G1logic_2=[ones(1,N)];  S1logic_2=[ones(1,N)];  N1logic_2=[ones(1,N)];
alogic2_2=[zeros(1,N)]; blogic2_2=[ones(1,N)];  G2logic_2=[ones(1,N)];  S2logic_2=[ones(1,N)];  N2logic_2=[ones(1,N)];
alogic3_2=[zeros(1,N)]; blogic3_2=[ones(1,N)];  G3logic_2=[ones(1,N)];  S3logic_2=[ones(1,N)];  N3logic_2=[ones(1,N)];
alogic0_2=[zeros(1,N)]; blogic0_2=[ones(1,N)];  G0logic_2=[ones(1,N)];  S0logic_2=[ones(1,N)];  N0logic_2=[ones(1,N)];

%..................... vehiculo 3 ........................
alogic1_3=[zeros(1,N)]; blogic1_3=[ones(1,N)];  G1logic_3=[ones(1,N)];  S1logic_3=[ones(1,N)];  N1logic_3=[ones(1,N)];
alogic2_3=[zeros(1,N)]; blogic2_3=[ones(1,N)];  G2logic_3=[ones(1,N)];  S2logic_3=[ones(1,N)];  N2logic_3=[ones(1,N)];
alogic3_3=[zeros(1,N)]; blogic3_3=[ones(1,N)];  G3logic_3=[ones(1,N)];  S3logic_3=[ones(1,N)];  N3logic_3=[ones(1,N)];
alogic0_3=[zeros(1,N)]; blogic0_3=[ones(1,N)];  G0logic_3=[ones(1,N)];  S0logic_3=[ones(1,N)];  N0logic_3=[ones(1,N)];

%..................... vehiculo 4 ........................
alogic1_4=[zeros(1,N)]; blogic1_4=[ones(1,N)];  G1logic_4=[ones(1,N)];  S1logic_4=[ones(1,N)];  N1logic_4=[ones(1,N)];
alogic2_4=[zeros(1,N)]; blogic2_4=[ones(1,N)];  G2logic_4=[ones(1,N)];  S2logic_4=[ones(1,N)];  N2logic_4=[ones(1,N)];
alogic3_4=[zeros(1,N)]; blogic3_4=[ones(1,N)];  G3logic_4=[ones(1,N)];  S3logic_4=[ones(1,N)];  N3logic_4=[ones(1,N)];
alogic0_4=[zeros(1,N)]; blogic0_4=[ones(1,N)];  G0logic_4=[ones(1,N)];  S0logic_4=[ones(1,N)];  N0logic_4=[ones(1,N)];

%..................... vehiculo 5 ........................
alogic1_5=[zeros(1,N)]; blogic1_5=[ones(1,N)];  G1logic_5=[ones(1,N)];  S1logic_5=[ones(1,N)];  N1logic_5=[ones(1,N)];
alogic2_5=[zeros(1,N)]; blogic2_5=[ones(1,N)];  G2logic_5=[ones(1,N)];  S2logic_5=[ones(1,N)];  N2logic_5=[ones(1,N)];
alogic3_5=[zeros(1,N)]; blogic3_5=[ones(1,N)];  G3logic_5=[ones(1,N)];  S3logic_5=[ones(1,N)];  N3logic_5=[ones(1,N)];
alogic0_5=[zeros(1,N)]; blogic0_5=[ones(1,N)];  G0logic_5=[ones(1,N)];  S0logic_5=[ones(1,N)];  N0logic_5=[ones(1,N)];

% ..........historial de las predicciones
vp1hist=[];   zp1hist=[];
vp2hist=[];   zp2hist=[];
vp3hist=[];   zp3hist=[];
vp4hist=[];   zp4hist=[];
vp5hist=[];   zp5hist=[];



i=0;
%% online optimization
%------condiciones iniciales----------
vel= [10; 10; 10; 10; 10; 0];% velociodad inicial
Vdes=[30; 80; 80; 80; 80; 0]; %velocidad deseada

zel= [2; 5; 5; 3; 1];   %carril inicial
% Zdes=[1; 5; 3; 4; 2];   %carril deseado
acel=[0 0 0 0 0 0]';      %aceleracion inicial
zel_obs = [3];

% %------condiciones iniciales----------
% vel= [20; 20; 20; 20; 20; 0];% velociodad inicial
% Vdes=[30; 70; 70; 70; 70; 0]; %velocidad deseada
% 
% zel= [3; 4; 2; 1; 4];   %carril inicial
Zdes=[2; 1; 3; 3; 1];   %carril deseado
% acel=[10 10 10 10 10 0]';      %aceleracion inicial
% zel_obs = [2];
%---distancia inicial de cada agente
d1i = [-40 -100 -70 -120 200]';




p_optima = ( Vdes(1)-Vdes(1) )'*Q*( Vdes(1)-Vdes(1) ) + (Zdes(1) - Zdes(1))'*R*(Zdes(1) - Zdes(1));
epsilon = 10^(-12);

% hold on
vhist = vel;
zhist = zel;
ahist = acel;
dhist = d1i;
mpciter = 0;


sim_tim = 13; % Maximum simulation time
tic

% for ii = 1 : 30
while ( vel-Vdes )'*Q*( vel-Vdes ) + (zel - Zdes)'*R*(zel - Zdes) - p_optima > epsilon && mpciter < sim_tim
    i=i+1;

    %.........................      solver vehiculo 1       ............................
    
    inputs1 = {Vdes(1), Zdes(1), vel(1), zel(1), ...
        vel(2), zel(2), d1i(1), alogic1_1, blogic1_1, S1logic_1, N1logic_1,...
        vel(3), zel(3), d1i(2), alogic2_1, blogic2_1, S2logic_1, N2logic_1};
%         vel(4), zel(4), d1i(3), alogic3_1, blogic3_1, S3logic_1, N3logic_1};
%                zel_obs, d1i(5),                       S0logic_1, N0logic_1};
    [solutions1,diagnostics] = controller2{inputs1}; %Tipo de controlador de segun el # de nodos
    
    A =  solutions1{1};         acel(1) = A(:,1);
    Z =  solutions1{2};         zel(1)=Z(:,2);                  zp1hist=[zp1hist; Z];
    V =  solutions1{3};         vp1hist = [vp1hist; V];
    Aa = solutions1{4};         alogic1_1 = Aa;
    B =  solutions1{5};         blogic1_1 = B;
    S =  solutions1{6};         S1logic_1 = [S(2:N) 1];
    Nn = solutions1{7};         N1logic_1 = Nn;
    
    Aa2 = solutions1{8};        alogic2_1 = Aa2;
    B2 = solutions1{9};         blogic2_1 = B2;
    S2 = solutions1{10};        S2logic_1 = [S2(2:N) 1];
    Nn2 = solutions1{11};       N2logic_1 = Nn2;
    
%     Aa3 = solutions1{12};       alogic3_1 = Aa3;
%     B3 = solutions1{13};        blogic3_1 = B3;
%     S3 = solutions1{14};        S3logic_1 = [S3(2:N) 1];
%     Nn3 = solutions1{15};       N3logic_1 = Nn3;
%     
%     S0 = solutions1{16};        S0logic_1 = [S0(2:N) 1];
%     Nn0 = solutions1{17};       N0logic_1 = Nn0;
    
    if diagnostics == 1
        error('you are close, keep trying 1');
    end
    
    %.........................      solver vehiculo 2       ............................
    
    inputs2 = {Vdes(2), Zdes(2),  vel(2), zel(2),...
        vel(1), zel(1), -d1i(1),          alogic1_2, blogic1_2, S1logic_2, N1logic_2,...
        vel(4), zel(4), (-d1i(1)+d1i(3)), alogic2_2, blogic2_2, S2logic_2, N2logic_2,...
        vel(5), zel(5), (-d1i(1)+d1i(4)), alogic3_2, blogic3_2, S3logic_2, N3logic_2};
    [solutions2,diagnostics] = controller3{inputs2};
    
    A = solutions2{1};      acel(2) = A(:,1);
    Z = solutions2{2};      zel(2)=Z(:,2);                  zp2hist=[zp2hist; Z];
    V = solutions2{3};      vp2hist =  [vp2hist; V];
    Aa = solutions2{4};         alogic1_2 = Aa;
    B = solutions2{5};          blogic1_2 = B;
    S = solutions2{6};          S1logic_2 = [S(2:N) 1];
    Nn = solutions2{7};         N1logic_2 = Nn;
    
    Aa2 = solutions2{8};        alogic2_2 = Aa2;
    B2 = solutions2{9};         blogic2_2 = B2;
    S2 = solutions2{10};        S2logic_2 = [S2(2:N) 1];
    Nn2 = solutions2{11};       N2logic_2 = Nn2;
    
    Aa3 = solutions2{12};       alogic3_2 = Aa3;
    B3 = solutions2{13};        blogic3_2 = B3;
    S3 = solutions2{14};        S3logic_2 = [S3(2:N) 1];
    Nn3 = solutions2{15};       N3logic_2 = Nn3;
    
    if diagnostics == 1
        error('you are close, keep trying 2');
    end
    
    
    
    
    %.........................      solver vehiculo 3       ............................
    
    inputs3 = {Vdes(3) , Zdes(3) , vel(3) , zel(3) , ...
        vel(1) , zel(1)  , -d1i(2) ,        alogic1_3 , blogic1_3  , S1logic_3 , N1logic_3,...
        vel(4) , zel(4)  , -d1i(2)+d1i(3) , alogic2_3 , blogic2_3  , S2logic_3 , N2logic_3,...
        vel(5) , zel(5)  , -d1i(2)+d1i(4) , alogic3_3 , blogic3_3  , S3logic_3 , N3logic_3};
    [solutions3,diagnostics] = controller3{inputs3};
    
    A = solutions3{1};      acel(3) = A(:,1);
    Z = solutions3{2};      zel(3)=Z(:,2);                    zp3hist=[zp3hist; Z];
    V = solutions3{3};      vp3hist = [vp3hist; V];
    Aa1 = solutions3{4};        alogic1_3 = Aa1;
    B1  = solutions3{5};        blogic1_3 = B1;
    S1  = solutions3{6};        S1logic_3 = [S1(2:N) 1];
    Nn1 = solutions3{7};        N1logic_3 = Nn1;
    
    Aa2 = solutions3{8};        alogic2_3 = Aa2;
    B2 = solutions3{9};         blogic2_3 = B2;
    S2 = solutions3{10};        S2logic_3 = [S2(2:N) 1];
    Nn2 = solutions3{11};       N2logic_3 = Nn2;
    
    Aa3 = solutions3{12};       alogic3_3 = Aa3;
    B3 = solutions3{13};        blogic3_3 = B3;
    S3 = solutions3{14};        S3logic_3 = [S3(2:N) 1];
    Nn3 = solutions3{15};       N3logic_3 = Nn3;
    
    if diagnostics == 1
        error('you are close, keep trying 3');
    end
    
    %.........................      solver vehiculo 4       ............................
    
    inputs4 = {Vdes(4) , Zdes(4) , vel(4) , zel(4) , ...
        vel(2), zel(2), (-d1i(3)+d1i(1)), alogic1_4, blogic1_4, S1logic_4, N1logic_4,...
        vel(3), zel(3), (-d1i(3)+d1i(2)), alogic2_4, blogic2_4, S2logic_4, N2logic_4,...
        vel(5), zel(5), (-d1i(3)+d1i(4)), alogic3_4, blogic3_4, S3logic_4, N3logic_4};
    [solutions4,diagnostics] = controller3{inputs4};
    
    A = solutions4{1};      acel(4) = A(:,1);
    Z = solutions4{2};      zel(4)=Z(:,2);                    zp4hist=[zp4hist; Z];
    V = solutions4{3};      vp4hist = [vp4hist; V];
    Aa1 = solutions4{4};        alogic1_4 = Aa1;
    B1  = solutions4{5};        blogic1_4 = B1;
    S1  = solutions4{6};        S1logic_4 = [S1(2:N) 1];
    Nn1 = solutions4{7};        N1logic_4 = Nn1;
    
    Aa2 = solutions4{8};        alogic2_4 = Aa2;
    B2 = solutions4{9};         blogic2_4 = B2;
    S2 = solutions4{10};        S2logic_4 = [S2(2:N) 1];
    Nn2 = solutions4{11};       N2logic_4 = Nn2;
    
    Aa3 = solutions4{12};       alogic3_4 = Aa3;
    B3 = solutions4{13};        blogic3_4 = B3;
    S3 = solutions4{14};        S3logic_4 = [S3(2:N) 1];
    Nn3 = solutions4{15};       N3logic_4 = Nn3;
    
    if diagnostics == 1
        error('you are close, keep trying 4');
    end
    
    
    
    %.........................      solver vehiculo 5       ............................
    
    inputs5 = {Vdes(5), Zdes(5), vel(5), zel(5), ...
        vel(2), zel(2), (d1i(1)-d1i(4)), alogic1_5, blogic1_5, S1logic_5, N1logic_5,...
        vel(3), zel(3), (d1i(2)-d1i(4)), alogic2_5, blogic2_5, S2logic_5, N2logic_5,...
        vel(4), zel(4), (d1i(3)-d1i(4)), alogic3_5, blogic3_5, S3logic_5, N3logic_5};
    [solutions5,diagnostics] = controller3{inputs5};
    
    A = solutions5{1};      acel(5) = A(:,1);
    Z = solutions5{2};      zel(5)=Z(:,2);                    zp5hist=[zp5hist; Z];
    V = solutions5{3};      vp5hist = [vp5hist; V];
    Aa1 = solutions5{4};        alogic1_5 = Aa1;
    B1  = solutions5{5};        blogic1_5 = B1;
    S1  = solutions5{6};        S1logic_5 = [S1(2:N) 1];
    Nn1 = solutions5{7};        N1logic_5 = Nn1;
    
    Aa2 = solutions5{8};        alogic2_5 = Aa2;
    B2 = solutions5{9};         blogic2_5 = B2;
    S2 = solutions5{10};        S2logic_5 = [S2(2:N) 1];
    Nn2 = solutions5{11};       N2logic_5 = Nn2;
    
    Aa3 = solutions5{12};       alogic3_5 = Aa3;
    B3 = solutions5{13};        blogic3_5 = B3;
    S3 = solutions5{14};        S3logic_5 = [S3(2:N) 1];
    Nn3 = solutions5{15};       N3logic_5 = Nn3;
    
    if diagnostics == 1
        error('you are close, keep trying 5');
    end
    
    
    
    
    %----------------------------------------------------------------------
    
    d1i = d1i + T*(vel(2:( size(vel,1) )) - ones((size(vel,1)-1),1)*vel(1));
    d2i = [-d1i(1); -d1i(1)+d1i(2)];
    
    vel = vel + T*acel;
    vhist = [vhist vel];
    zhist = [zhist zel];
    ahist = [ahist acel];
    dhist = [dhist d1i];
   
    
    mpciter
    mpciter = mpciter + 1;
end
toc

disp("it's done")

vphist=cat(3, vp1hist , vp2hist, vp3hist, vp4hist, vp5hist);
zphist=cat(3, zp1hist , zp2hist, zp3hist, zp4hist, zp5hist);

Draw_object(vhist,zhist,vphist,zphist,dhist,T,delay_time)
% save('myFile5.mat','vhist','zhist','vphist','zphist','dhist','T')