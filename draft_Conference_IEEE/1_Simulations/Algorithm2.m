
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

v_2 = sdpvar(1,nv);  %velocidad del otro vehculo
z_2 = sdpvar(1,nv);  %carril del vehiculo j

dis12 = sdpvar(ones(1,N+1),ones(1,N+1));  %distancia entre vehiculo 1 y 2

a1 = binvar(ones(1,N),ones(1,N));    
b1 = binvar(ones(1,N),ones(1,N));   
g1 = binvar(ones(1,N),ones(1,N));      
s1 = binvar(ones(1,N),ones(1,N));    
n1 = binvar(ones(1,N),ones(1,N));  

Aa1 = binvar( 1,N ); 
Bb1 = binvar( 1,N ); 
Gg1 = binvar( 1,N ); 
Ss1 = binvar( 1,N );
Nn1 = binvar( 1,N );

D1 = binvar(3*ones(1,N),ones(1,N));  
B1 = binvar(2*ones(1,N),ones(1,N));  
G1 = binvar(3*ones(1,N),ones(1,N));  
S1 = binvar(5*ones(1,N),ones(1,N));   
N1 = binvar(3*ones(1,N),ones(1,N));  

p_a = sdpvar(1);
p_z = sdpvar(1);
constraints = [];

constraints = [constraints,  diff([p_z z{1}]) == 0];
objective   = 0;

%% creacion de funcion objetivo y restricciones
for k = 1:N
 objective = objective+( v{k+1}-Vd )'*Q*( v{k+1}-Vd ) + (z{k+1} - Zd)'*R*(z{k+1} - Zd); % calculate obj

  % Feasible region
    constraints = [constraints,1 <=    z{k+1}     <= L,
                               1 <=    z_2      <= L,       %tome valores posibles
                                0<=    v{k+1}   <= V_max,   %no exceda las velocidades 
                         z{k}-[1]<=    z{k+1}   <=z{k}+[1], %paso de un carril
                           -A_max<=    a{k}     <= A_max];
    
    constraints = [constraints, [1 <= z{1}    <=   L   ]];

                       
%     constraints = [constraints, ll{k}+lr{k} <=1 ];
                       
                       
    constraints = [constraints, -1 <= [z{k+1} - z{k}] <= 1];
    constraints = [constraints, v{k+1} == v{k}+T*a{k}];             %velocidad futura
%     constraints = [constraints, z{k+1} == z{k}+ ll{k} - lr{k}];       %carril futuro
   
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


%.........................Gamma...............................
% 
% constraints = [constraints, [G1{k}(1)+G1{k}(2)+G1{k}(3)==1], 
%               implies( G1{k}(1), [  z_2-z{k+1} <= -0.1 ,       g1{k}==0 ]);
%               implies( G1{k}(2), [  -0.1 <= z_2-z{k+1} <=0.1 , g1{k}==1 ]);
%               implies( G1{k}(3), [  0.1 <= z_2-z{k+1} ,        g1{k}==0 ]) ];   
%           
%  constraints = [constraints,  Aa1(k)*Gg1(k)*( Bb1(k) * ( T*(v_2-v{k}) + dis12{k}) + (1-Bb1(k))*(T*(v_2-v{k}) + dis12{k} ))<=0];
% constraints = [constraints, 1<=z{k+1}<=L ];           

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
%% solver definition   

parameters_in = { Vd , Zd , v{1} , p_z , ...
                            v_2 , z_2 , dis12{1} , Aa1 , Bb1 , Ss1 , Nn1};%, Gg1
                         
                            
solutions_out = {[a{:}], [z{:}], [v{:}], [a1{:}],  [b1{:}] ,  [s1{:}], [n1{:}] };%[g1{:}],
%                                          [a4], [dis15{:}], [b4], [g4], [z4], [n4]};

controller1 = optimizer(constraints, objective , sdpsettings('solver','gurobi'),parameters_in,solutions_out);
%------condiciones iniciales----------
vel= [10; 0; 10];% velociodad inicial
Vdes=[60; 50; 10]; %velocidad deseada

zel= [2; 3; 5]; %carril inicial
Zdes=[4; 2; 5]; %carril deseado

%---distancia inicial de cada agente
d1i = [10; 0];
acel=[0 0 0]';

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
 
 

% ..........historial de las predicciones 
 vp1hist=[];  
 zp1hist=[];   
 s1hist_1=[]; 
 
 vp2hist=[];  
 zp2hist=[];   
 s1hist_2=[];
 
p_optima = ( Vdes(1)-Vdes(1) )'*Q*( Vdes(1)-Vdes(1) ) + (Zdes(1) - Zdes(1))'*R*(Zdes(1) - Zdes(1));
epsilon = 10^(-12);
 
i=0;

 time=20;
 tic
% for i = 1 : time
while ( vel-Vdes )'*Q*( vel-Vdes ) + (zel - Zdes)'*R*(zel - Zdes) - p_optima > epsilon
 i=i+1;
%.........................      solver vehiculo 1       ............................

j(5)-j(5.2)
        inputs = {Vdes(1) , Zdes(1) , vel(1) , zel(1) , ...
                   vel(2) , zel(2)  , d1i(1) , alogic1_1 , blogic1_1  , S1logic_1 , N1logic_1}; %G1logic_1
    [solutions1,diagnostics] = controller1{inputs};    
     
    A = solutions1{1};      acel(1) = A(:,1);
    Z = solutions1{2};      zel(1)=Z(:,2);                  zp1hist=[zp1hist; Z];
    V = solutions1{3};      vp1hist = [vp1hist; V];
    Aa = solutions1{4};     a1hist_1 = [a1hist_1; Aa];      alogic1_1 = Aa;
    B = solutions1{5};      b1hist_1 = [b1hist_1; B];       blogic1_1 = B;
    S = solutions1{6};      s1hist_1 = [s1hist_1; S];       S1logic_1 = [S(2:N) 1];
    Nn = solutions1{7};     n1hist_1 = [n1hist_1; Nn];      N1logic_1 = Nn;

%     Gg1 = solutions1{6};    g1hist_1=[g1hist_1 Gg1];        G1logic_1=Gg1;

    if diagnostics == 1
        error('you are close, keep trying 1');
    end   
% end
% 
%  
% while ( vel-Vdes )'*Q*( vel-Vdes ) + (zel - Zdes)'*R*(zel - Zdes) - p_optima > epsilon  
%.........................      solver vehiculo 2       ............................

        inputs = {Vdes(2) , Zdes(2) , vel(2) , zel(2) , ...
                   vel(1) , zel(1)  , -d1i(1) , alogic1_2 , blogic1_2  , S1logic_2 , N1logic_2}; %G1logic_1
    [solutions1,diagnostics] = controller1{inputs};    
     
    A = solutions1{1};      acel(2) = A(:,1);
    Z = solutions1{2};      zel(2)=Z(:,2);                  zp2hist=[zp2hist; Z];
    V = solutions1{3};      vp2hist = [vp1hist; V];
    Aa = solutions1{4};     a1hist_2 = [a1hist_2; Aa];      alogic1_2 = Aa;
    B = solutions1{5};      b1hist_2 = [b1hist_2; B];       blogic1_2 = B;
    S = solutions1{6};      s1hist_2 = [s1hist_2; S];       S1logic_2 = [S(2:N) 1];
    Nn = solutions1{7};     n1hist_2 = [n1hist_2; Nn];      N1logic_2 = Nn;

%     Gg1 = solutions1{6};    g1hist_1=[g1hist_1 Gg1];        G1logic_1=Gg1;

    if diagnostics == 1
        error('you are close, keep trying 2');
    end
    
    
    
    
    
    

    d1i = d1i+T*(vel(2:( size(vel,1) ))-ones((size(vel,1)-1),1)*vel(1));
%     d2i = [-d1i(1); -d1i(1)+d1i(2); -d1i(1)+d1i(3)];
    
    vel = vel+T*acel;
    vhist = [vhist vel];
    zhist = [zhist zel];
    ahist = [ahist acel];
    dhist = [dhist d1i];

%     pause(0.05)   

end
toc
% vp2hist = [vel(2)*ones(i+2,N+1)];
% zp2hist = [zel(2)*ones(i+2,N+1)];
vp3hist = [vel(3)*ones(i,N+1)];
zp3hist = [zel(3)*ones(i,N+1)];


Draw_basico(vhist,zhist,...
                            vp1hist,zp1hist,...
                            vp2hist,zp2hist,...
                            vp3hist,zp3hist,...
                                                dhist,T,N)

