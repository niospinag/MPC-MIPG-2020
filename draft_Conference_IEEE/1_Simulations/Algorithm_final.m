
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
nx = 1; % Number of agents
nu = 1; % Number of inputs
nv=1; %numero de vehiculos sin el agente no cooperativo
% MPC data
Q = 1*eye(1);
R = 10*eye(1);
N = 5;%horizon
T = 0.5; %[s]
Ds=15;%Safety distance
Dl=15; %lateral distance
V_max=80;
A_max=10;
L=6;%number of lanes
Mmax=L-1;
mmin=-L+1;
% Vd = 12;
% Zd = 1;

%------------estados deseados-----------
 Zd = sdpvar(1,1);%carril deseado
 Vd = sdpvar(1,1);%velocidad deseada

% -------------vehiculo i---------------
v = sdpvar(ones(1,N+1),ones(1,N+1)); %velocidad del vehiculo actual
a = sdpvar(ones(1,N+1),ones(1,N+1)); %aceleracion actual del vehiculo
z = sdpvar(ones(1,N+1),ones(1,N+1)); %carril actual
ll = binvar(ones(1,N+1),ones(1,N+1)); %carril actual
lr = binvar(ones(1,N+1),ones(1,N+1)); %carril actual

v_2 = sdpvar(1,nv);  v_3 = sdpvar(1,nv);  %velocidad del otro vehculo
z_2 = sdpvar(1,nv);  z_3 = sdpvar(1,nv);  %carril del vehiculo j

dis12 = sdpvar(ones(1,N+1),ones(1,N+1));  %distancia entre vehiculo 1 y 2
dis13 = sdpvar(ones(1,N+1),ones(1,N+1));  %distancia entre vehiculo 1 y 2

a1 = binvar(ones(1,N),ones(1,N));    a2 = binvar(ones(1,N),ones(1,N));    
b1 = binvar(ones(1,N),ones(1,N));    b2 = binvar(ones(1,N),ones(1,N));    
g1 = binvar(ones(1,N),ones(1,N));    g2 = binvar(ones(1,N),ones(1,N));    
s1 = binvar(ones(1,N),ones(1,N));    s2 = binvar(ones(1,N),ones(1,N));    
n1 = binvar(ones(1,N),ones(1,N));    n2 = binvar(ones(1,N),ones(1,N));    

Aa1 = binvar( 1,N );                Aa2 = binvar( 1,N );                
Bb1 = binvar( 1,N );                Bb2 = binvar( 1,N );                
Gg1 = binvar( 1,N );                Gg2 = binvar( 1,N );                
Ss1 = binvar( 1,N );                Ss2 = binvar( 1,N );                
Nn1 = binvar( 1,N );                Nn2 = binvar( 1,N );                

D1 = binvar(3*ones(1,N),ones(1,N));  D2 = binvar(3*ones(1,N),ones(1,N));  
B1 = binvar(2*ones(1,N),ones(1,N));  B2 = binvar(2*ones(1,N),ones(1,N));  
G1 = binvar(3*ones(1,N),ones(1,N));  G2 = binvar(3*ones(1,N),ones(1,N));  
S1 = binvar(5*ones(1,N),ones(1,N));  S2 = binvar(5*ones(1,N),ones(1,N));  
N1 = binvar(3*ones(1,N),ones(1,N));  N2 = binvar(3*ones(1,N),ones(1,N));  

p_a = sdpvar(1);
p_z = sdpvar(1);


%% making the optimizer with 1 node
constraints = [];
constraints = [constraints,  diff([p_z z{1}]) == 0];
objective   = 0;

for k = 1:N
 objective = objective+( v{k+1}-Vd )'*Q*( v{k+1}-Vd ) + (z{k+1} - Zd)'*R*(z{k+1} - Zd); % calculate obj

  % Feasible region
    constraints = [constraints,1 <=    z{k+1}     <= L,
                               1 <=    z_2      <= L,       %tome valores posibles
                                0<=    v{k+1}   <= V_max,   %no exceda las velocidades 
                         z{k}-[1]<=    z{k+1}   <=z{k}+[1], %paso de un carril
                           -A_max<=    a{k}     <= A_max];
    
    constraints = [constraints, [1 <= z{1}    <=   L   ]];

    constraints = [constraints, -1 <= [z{k+1} - z{k}] <= 1];
    constraints = [constraints, v{k+1} == v{k}+T*a{k}];             %velocidad futura

   
% ---------------------------------------vehiculo 2-------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, -100000  <=  dis12{k+1} <= 100000,...
                                  mmin  <= z_2-z{k+1}  <= Mmax];
    constraints = [constraints, dis12{k+1} == dis12{k} + T*(v_2-v{k})];
    constraints = [constraints, [dis12{1} <= 100000]]; 
%.........................alpha...............................

constraints = [constraints, [D1{k}(1)+D1{k}(2)+D1{k}(3)==1],... 
              implies( D1{k}(1), [  z_2-z{k} <=-0.1 ,       a1{k}==0   ]);
              implies( D1{k}(2), [  -0.1<= z_2-z{k} <=0.1,  a1{k}==1   ]);
              implies( D1{k}(3), [  0.1 <= z_2-z{k},        a1{k}==0   ]) ];
          
%................................... Beta ....................................
constraints = [constraints, [sum(B1{k})==1],... 
              implies(B1{k}(1),[ dis12{k} >=0,     b1{k}==1]);
              implies(B1{k}(2),[ dis12{k} <=0,     b1{k}==0]) ];

          
constraints = [constraints,  Aa1(k)*( Bb1(k) *(Ds - dis12{k+1}) + (1-Bb1(k)) * (Ds + dis12{k+1})) <= 0 ];

% %.........................Lateral distance...............................

constraints = [constraints, [sum(S1{k})==1], 
              implies( S1{k}(1), [ s1{k} == 0,        z_2-z{k} <= -1.1 ]  );
              implies( S1{k}(2), [ s1{k} == 1,        z_2-z{k} == -1 ]  );
              implies( S1{k}(3), [ s1{k} == 0,   -0.9 <= z_2-z{k} <= 0.9 ]  );
              implies( S1{k}(4), [ s1{k} == 1,         z_2-z{k} == 1 ]  );
              implies( S1{k}(5), [ s1{k} == 0,      1.1 <= z_2-z{k} ]) ]  ;   

%.........................lateral safety distance...............................
constraints = [constraints, sum(N1{k})==1, 
              implies( N1{k}(1), [        dis12{1} <= -Dl ,     n1{k}==0 ] );
              implies( N1{k}(2), [  -Dl<= dis12{1} <= Dl,       n1{k}==1 ] );
              implies( N1{k}(3), [   Dl<= dis12{1}  ,           n1{k}==0 ] ) ];   
 
          
constraints = [constraints, Ss1(k)*(Nn1(k))*(z{k+1}-z{k})==0];
%................................................................................


    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
    
end
% objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
%% solver definition 2 node  

parameters_in = { Vd , Zd , v{1} , p_z , ...
                            v_2 , z_2 , dis12{1} , Aa1 , Bb1 , Ss1 , Nn1};%, Gg1
                         
                            
solutions_out = {[a{:}], [z{:}], [v{:}], [a1{:}],  [b1{:}] ,  [s1{:}], [n1{:}] };%[g1{:}],
%                                          [a4], [dis15{:}], [b4], [g4], [z4], [n4]};

controller1 = optimizer(constraints, objective , sdpsettings('solver','gurobi'),parameters_in,solutions_out);

%% making the optimizer with 3 node
constraints = [];
constraints = [constraints,  diff([p_z z{1}]) == 0];
objective   = 0;

for k = 1:N
 objective = objective+( v{k+1}-Vd )'*Q*( v{k+1}-Vd ) + (z{k+1} - Zd)'*R*(z{k+1} - Zd); % calculate obj

  % Feasible region
    constraints = [constraints,1 <=    z{k+1}     <= L,
                               1 <=    z_2      <= L,   
                               1 <=    z_3      <= L,   %tome valores posibles
                                0<=    v{k+1}   <= V_max,   %no exceda las velocidades 
                         z{k}-[1]<=    z{k+1}   <=z{k}+[1], %paso de un carril
                           -A_max<=    a{k}     <= A_max];
    
    constraints = [constraints, [1 <= z{1}    <=   L   ]];

    constraints = [constraints, -1 <= [z{k+1} - z{k}] <= 1];
    constraints = [constraints, v{k+1} == v{k}+T*a{k}];             %velocidad futura

   
% ---------------------------------------vehiculo 2-------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, -100000  <=  dis12{k+1} <= 100000,...
                                  mmin  <= z_2-z{k+1}  <= Mmax];
    constraints = [constraints, dis12{k+1} == dis12{k} + T*(v_2-v{k})];
    constraints = [constraints, [dis12{1} <= 100000]]; 
%.........................alpha...............................

constraints = [constraints, [D1{k}(1)+D1{k}(2)+D1{k}(3)==1],... 
              implies( D1{k}(1), [  z_2-z{k} <=-0.1 ,       a1{k}==0   ]);
              implies( D1{k}(2), [  -0.1<= z_2-z{k} <=0.1,  a1{k}==1   ]);
              implies( D1{k}(3), [  0.1 <= z_2-z{k},        a1{k}==0   ]) ];
          
%................................... Beta ....................................
constraints = [constraints, [sum(B1{k})==1],... 
              implies(B1{k}(1),[ dis12{k} >=0,     b1{k}==1]);
              implies(B1{k}(2),[ dis12{k} <=0,     b1{k}==0]) ];

          
constraints = [constraints,  Aa1(k)*( Bb1(k) *(Ds - dis12{k+1}) + (1-Bb1(k)) * (Ds + dis12{k+1})) <= 0 ];

% %.........................Lateral distance...............................

constraints = [constraints, [sum(S1{k})==1], 
              implies( S1{k}(1), [ s1{k} == 0,        z_2-z{k} <= -1.1 ]  );
              implies( S1{k}(2), [ s1{k} == 1,        z_2-z{k} == -1 ]  );
              implies( S1{k}(3), [ s1{k} == 0,   -0.9 <= z_2-z{k} <= 0.9 ]  );
              implies( S1{k}(4), [ s1{k} == 1,         z_2-z{k} == 1 ]  );
              implies( S1{k}(5), [ s1{k} == 0,      1.1 <= z_2-z{k} ]) ]  ;   

%.........................lateral safety distance...............................
constraints = [constraints, sum(N1{k})==1, 
              implies( N1{k}(1), [        dis12{1} <= -Dl ,     n1{k}==0 ] );
              implies( N1{k}(2), [  -Dl<= dis12{1} <= Dl,       n1{k}==1 ] );
              implies( N1{k}(3), [   Dl<= dis12{1}  ,           n1{k}==0 ] ) ];   
 
          
constraints = [constraints, Ss1(k)*(Nn1(k))*(z{k+1}-z{k})==0];



% --------------------------------------- vehiculo 3 -------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds -----------------

    constraints = [constraints, -100000  <=  dis13{k+1} <= 100000,...
                                   mmin  <= z_3-z{k+1}  <= Mmax];
    constraints = [constraints, dis13{k+1} == dis13{k} + T*(v_2-v{k})];
    constraints = [constraints, [dis13{1} <= 100000]]; 
%.........................alpha...............................

constraints = [constraints, [D2{k}(1)+D2{k}(2)+D2{k}(3)==1],... 
              implies( D2{k}(1), [  z_3-z{k} <=-0.1 ,       a2{k}==0   ]);
              implies( D2{k}(2), [  -0.1<= z_3-z{k} <=0.1,  a2{k}==1   ]);
              implies( D2{k}(3), [  0.1 <= z_3-z{k},        a2{k}==0   ]) ];
          
%................................... Beta ....................................
constraints = [constraints, [sum(B2{k})==1],... 
              implies(B2{k}(1),[ dis13{k} >=0,     b2{k}==1]);
              implies(B2{k}(2),[ dis13{k} <=0,     b2{k}==0]) ];

          
constraints = [constraints,  Aa2(k)*( Bb2(k) *(Ds - dis13{k+1}) + (1-Bb2(k)) * (Ds + dis13{k+1})) <= 0 ];

% %.........................Lateral distance...............................

constraints = [constraints, [sum(S2{k})==1], 
              implies( S2{k}(1), [ s2{k} == 0,        z_3-z{k} <= -1.1 ]  );
              implies( S2{k}(2), [ s2{k} == 1,        z_3-z{k} == -1 ]  );
              implies( S2{k}(3), [ s2{k} == 0,   -0.9 <= z_3-z{k} <= 0.9 ]  );
              implies( S2{k}(4), [ s2{k} == 1,         z_3-z{k} == 1 ]  );
              implies( S2{k}(5), [ s2{k} == 0,    1.1 <= z_3-z{k} ]) ]  ;   

%.........................lateral safety distance...............................
constraints = [constraints, sum(N2{k})==1, 
              implies( N2{k}(1), [        dis13{1} <= -Dl ,     n2{k}==0 ] );
              implies( N2{k}(2), [  -Dl<= dis13{1} <= Dl,       n2{k}==1 ] );
              implies( N2{k}(3), [   Dl<= dis13{1}  ,           n2{k}==0 ] ) ];   
 
          
constraints = [constraints, Ss2(k)*(Nn2(k))*(z{k+1}-z{k})==0];


    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
    
end
% objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
%% solver definition  3 node

parameters_in = { Vd , Zd , v{1} , p_z , ...
                            v_2 , z_2 , dis12{1} , Aa1 , Bb1 , Ss1 , Nn1...
                            v_3 , z_3 , dis13{1} , Aa2 , Bb2 , Ss2 , Nn2};

solutions_out = {[a{:}], [z{:}], [v{:}], [a1{:}],  [b1{:}] ,  [s1{:}], [n1{:}],...
                                         [a2{:}],  [b2{:}] ,  [s2{:}], [n2{:}] };

controller2 = optimizer(constraints, objective , sdpsettings('solver','gurobi'),parameters_in,solutions_out);




%% initial condition

%------condiciones iniciales----------
vel= [10; 0; 30; 20];% velociodad inicial
Vdes=[60; 50; 50; 20]; %velocidad deseada

zel= [2; 3; 4; 5]; %carril inicial
Zdes=[4; 1; 5; 4]; %carril deseado

acel=[0 0 0 0]';
%---distancia inicial de cada agente
d1i = [80 60 50]';
% d1i = [10 0 30]';
d2i = [-d1i(1); -d1i(1)+d1i(2)];
% d3i = [-d1i(2); -d1i(2)+d1i(1); -d1i(2)+d1i(3)];


%define las condiciones iniciales que deben tener las variables
%logicas

 %.....................vehiculo 1..........................
 
 alogic1_1=[zeros(1,N)]; 
 blogic1_1=[ones(1,N)];  
 G1logic_1=[ones(1,N)];     
 S1logic_1=[ones(1,N)]; 
 N1logic_1=[ones(1,N)];   

 %..................... vehiculo 2 ..........................
 
 alogic1_2=[zeros(1,N)]; 
 blogic1_2=[ones(1,N)];  
 G1logic_2=[ones(1,N)];     
 S1logic_2=[ones(1,N)]; 
 N1logic_2=[ones(1,N)];   
 
 %..................... vehiculo 3 ..........................
 
 alogic1_3=[zeros(1,N)]; 
 blogic1_3=[ones(1,N)];  
 G1logic_3=[ones(1,N)];     
 S1logic_3=[ones(1,N)]; 
 N1logic_3=[ones(1,N)];   
 
 alogic2_3=[zeros(1,N)]; 
 blogic2_3=[ones(1,N)];  
 G2logic_3=[ones(1,N)];     
 S2logic_3=[ones(1,N)]; 
 N2logic_3=[ones(1,N)];   
 
  %..................... vehiculo 4 ..........................
 
 alogic1_4=[zeros(1,N)]; 
 blogic1_4=[ones(1,N)];  
 G1logic_4=[ones(1,N)];     
 S1logic_4=[ones(1,N)]; 
 N1logic_4=[ones(1,N)];   
 
 
 
% hold on
vhist = vel;
zhist = zel;
ahist = acel;
dhist = d1i;

 % ...historial variables logicas
 a1hist_1=[];
 b1hist_1=[];    
 g1hist_1=[];   
 n1hist_1=[];  
 
 a1hist_2=[];
 b1hist_2=[];    
 g1hist_2=[];   
 n1hist_2=[];  
 
  
 a1hist_3=[];    a2hist_3=[];   
 b1hist_3=[];    b2hist_3=[];    
 g1hist_3=[];    g2hist_3=[];   
 n1hist_3=[];    n2hist_3=[];   
 
 

% ..........historial de las predicciones 
 vp1hist=[];  
 zp1hist=[];   
 s1hist_1=[]; 
 
 vp2hist=[];  
 zp2hist=[];   
 s1hist_2=[];
 
 vp3hist=[];  
 zp3hist=[];   
 s1hist_3=[];    s2hist_3=[];
 
 vp4hist=[];  
 zp4hist=[];   
 s1hist_4=[]; 
 
p_optima = ( Vdes(1)-Vdes(1) )'*Q*( Vdes(1)-Vdes(1) ) + (Zdes(1) - Zdes(1))'*R*(Zdes(1) - Zdes(1));
epsilon = 10^(-12);
 
i=0;

 time=20;
 tic
% for i = 1 : time
while ( vel-Vdes )'*Q*( vel-Vdes ) + (zel - Zdes)'*R*(zel - Zdes) - p_optima > epsilon
 i=i+1;
%.........................      solver vehiculo 1       ............................


        inputs = {Vdes(1) , Zdes(1) , vel(1) , zel(1) , ...
                   vel(2) , zel(2)  , d1i(1) , alogic1_1 , blogic1_1  , S1logic_1 , N1logic_1}; 
    [solutions1,diagnostics] = controller1{inputs};    
     
    A =  solutions1{1};         acel(1) = A(:,1);
    Z =  solutions1{2};         zel(1)=Z(:,2);                  zp1hist=[zp1hist; Z];
    V =  solutions1{3};         vp1hist = [vp1hist; V];
    Aa = solutions1{4};         alogic1_1 = Aa;
    B =  solutions1{5};         blogic1_1 = B;
    S =  solutions1{6};         S1logic_1 = [S(2:N) 1];
    Nn = solutions1{7};         N1logic_1 = Nn;

%     Gg1 = solutions1{6};    g1hist_1=[g1hist_1 Gg1];        G1logic_1=Gg1;

    if diagnostics == 1
        error('you are close, keep trying 1');
    end   

    
%.........................      solver vehiculo 2       ............................

        inputs = {Vdes(2) , Zdes(2) , vel(2) , zel(2) , ...
                   vel(1) , zel(1)  , -d1i(1) , alogic1_2 , blogic1_2  , S1logic_2 , N1logic_2}; %G1logic_1
    [solutions2,diagnostics] = controller1{inputs};    
     
    A = solutions2{1};      acel(2) = A(:,1);
    Z = solutions2{2};      zel(2)=Z(:,2);                  zp2hist=[zp2hist; Z];
    V = solutions2{3};      vp2hist =  [vp2hist; V];        
    Aa = solutions2{4};     alogic1_2 = Aa;
    B = solutions2{5};      blogic1_2 = B;
    S = solutions2{6};      S1logic_2 = [S(2:N) 1];
    Nn = solutions2{7};     N1logic_2 = Nn;
 
%     Gg1 = solutions1{6};    g1hist_1=[g1hist_1 Gg1];        G1logic_1=Gg1;

    if diagnostics == 1
        error('you are close, keep trying 2');
    end
    
    
    %.........................      solver vehiculo 3       ............................

        inputs = {Vdes(3) , Zdes(3) , vel(3) , zel(3) , ...
                   vel(1) , zel(1)  , d2i(1) , alogic1_3 , blogic1_3  , S1logic_3 , N1logic_3...
                   vel(2) , zel(2)  , d2i(2) , alogic2_3 , blogic2_3  , S2logic_3 , N2logic_3}; %G1logic_1
    [solutions3,diagnostics] = controller2{inputs};    
     
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
 
    if diagnostics == 1
        error('you are close, keep trying 3');
    end
    
    
      %.........................      solver vehiculo 4       ............................

%     parameters_in = { Vd , Zd , v{1} , p_z , ...
%                             v_2 , z_2 , dis12{1} , Aa1 , Bb1 , Ss1 , Nn1...
%                             v_3 , z_3 , dis13{1} , Aa2 , Bb2 , Ss2 , Nn2};
 
% solutions_out = {[a{:}], [z{:}], [v{:}], [a1{:}],  [b1{:}] ,  [s1{:}], [n1{:}],...
%                                          [a2{:}],  [b2{:}] ,  [s2{:}], [n2{:}] };
%     
        inputs = {Vdes(4) , Zdes(4) , vel(4) , zel(4) , ...
                   vel(2) , zel(2)  , [-d1i(3)+d1i(1)] , alogic1_4 , blogic1_4  , S1logic_4 , N1logic_4}; 
    [solutions4,diagnostics] = controller1{inputs};    
     
    A = solutions4{1};      acel(4) = A(:,1);
    Z = solutions4{2};      zel(4)=Z(:,2);                    zp4hist=[zp4hist; Z];
    V = solutions4{3};      vp4hist = [vp4hist; V];
    
    Aa1 = solutions4{4};        alogic1_4 = Aa1;
    B1  = solutions4{5};        blogic1_4 = B1;
    S1  = solutions4{6};        S1logic_4 = [S1(2:N) 1];
    Nn1 = solutions4{7};        N1logic_4 = Nn1;
    
 
    if diagnostics == 1
        error('you are close, keep trying 4');
    end
    
    
    
    
    
    
    

    d1i = d1i + T*(vel(2:( size(vel,1) )) - ones((size(vel,1)-1),1)*vel(1));
    d2i = [-d1i(1); -d1i(1)+d1i(2)];
    
    vel = vel+T*acel;
    vhist = [vhist vel];
    zhist = [zhist zel];
    ahist = [ahist acel];
    dhist = [dhist d1i];

%     pause(0.05)   

end
toc


vphist=cat(3, vp1hist , vp2hist, vp3hist, vp4hist);
zphist=cat(3, zp1hist , zp2hist, zp3hist, zp4hist);

% Draw_basico(vhist,zhist,...                     
%                             vp1hist,zp1hist,...
%                             vp2hist,zp2hist,...
%                             vp3hist,zp3hist,...
%                                                  dhist,T,N)


Draw_object(vhist,zhist,vphist,zphist,dhist,T)

