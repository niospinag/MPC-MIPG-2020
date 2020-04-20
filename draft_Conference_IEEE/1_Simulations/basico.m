
% %UNIVERSIDAD NACIONAL DE COLOMBIA
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
addpath(genpath('C:\gurobi901\win64\matlab'))%GUROBI
addpath(genpath('C:\Users\nesto\OneDrive\Documentos\YALMIP-master'))%yalmip



yalmip('clear')
%% PROGRAM 
% Model data
nx = 1; % Number of agents
nu = 1; % Number of inputs
nv=1; %numero de vehiculos sin el agente no cooperativo
% MPC data
Q = 4*eye(1);
R = 4*eye(1);
N = 10;%horizon
T = 0.5; %[s]
Ds=3;%Safety distance
Dl=15; %lateral distance
V_max=40;
A_max=7;
L=6;%number of lanes
Mmax=L-1;
mmin=-L+1;
e=1;
i=1;
% Vd = 12;
% Zd = 1;

%------------estados deseados-----------
 Zd = sdpvar(1,1);%carril deseado
 Vd = sdpvar(1,1);%velocidad deseada

% -------------vehiculo i---------------
v = sdpvar(ones(1,N+1),ones(1,N+1)); %velocidad del vehiculo actual
a = sdpvar(ones(1,N+1),ones(1,N+1)); %aceleracion actual del vehiculo
z = sdpvar(ones(1,N+1),ones(1,N+1)); %carril actual

v_2 = sdpvar(1,nv);  %velocidad del otro vehculo
z_2 = sdpvar(1,nv);  %carril del vehiculo j

dis12 = sdpvar(ones(1,N+1),ones(1,N+1));  %distancia entre vehiculo 1 y 2

a1 = binvar(ones(1,N),ones(1,N));    
b1 = binvar(ones(1,N),ones(1,N));   
g1 = binvar(1,nv);      
z1 = binvar(1,nv);    
n1 = binvar(1,nv);  

Aa1 = sdpvar(ones(1,N)); 
Bb1 = sdpvar(ones(1,N),ones(1,N));
Gg1 = binvar(1,N); 
Ss1 = binvar(1,N);
Nn1 = binvar(1,N);

D1 = binvar(3*ones(1,N),ones(1,N));  
B1 = binvar(2*ones(1,N),ones(1,N));  
G1 = binvar(3,1,nv);  
Z1 = binvar(5,1,nv);   
N1 = binvar(3,1,nv);  

p_a = sdpvar(1);
p_z = sdpvar(1);

constraints = -0.8 <= diff([p_a a{:}]) <= 1;
constraints = [constraints, -1 <= diff([p_z z{:}]) <= 1];
objective   = 0;

%% creacion de funcion objetivo y restricciones--------------------
for k = 1:N
 objective = objective+(v{k}-Vd)'*Q*(v{k}-Vd) + (z{k}-Zd)'*R*(z{k}-Zd); % calculate obj
  
  % Feasible region
    constraints = [constraints,1 <=    z{k}     <= L,
                               1 <=    z_2      <= L,%tome valores posibles
                               
                                0<=    v{k+1}   <= V_max,%no exceda las velocidades 
                         z{k}-[1]<=    z{k+1}   <=z{k}+[1], %paso de un carril
                           -A_max<=    a{k}     <= A_max];
                       
                       
                       
    constraints = [constraints, -1 <= diff([z{k+1} z{k}]) <= 1];
    constraints = [constraints, v{k+1} == v{k}+T*a{k}]; %velocidad futura
  
   
% ------------------------------------vehiculo 2-------------------------------    
% ------------------si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, -10000  <=  dis12{k+1} <= 100000,...
                                  mmin  <= z_2-z{k+1}  <= Mmax];
    constraints = [constraints, dis12{k+1} == dis12{k} + T*(v_2-v{k})];
    constraints = [constraints, [dis12{1} <= 100000]]; 
%.........................alpha...............................

constraints = [constraints, [D1{k}(1)+D1{k}(2)+D1{k}(3)==1],... 
              implies( D1{k}(1), [ a1{k}==0, z_2-z{k} <=-0.1 ]);
              implies( D1{k}(2), [ a1{k}==1, -0.1<= z_2-z{k} <=0.1 ]);
              implies( D1{k}(3), [ a1{k}==0, 0.1 <= z_2-z{k} ]) ];
          
%.........................Beta...............................
constraints = [constraints, [sum(B1{k})==1],... 
              implies(B1{k}(1),[ b1{k}==1, dis12{k} >=0 ]);
              implies(B1{k}(2),[ b1{k}==0, dis12{k} <=0 ]) ];



% %.........................Gamma...............................
% 
% constraints = [constraints, [G1(1,1,i)+G1(2,1,i)+G1(3,1,i)==1],... 
%               implies( G1(1,1,i), [ g1==0, z_2-z{k+1} <=-0.1 ]);
%               implies( G1(2,1,i), [ g1==1, -0.1<=z_2-z{k+1} <=0.1 ]);
%               implies( G1(3,1,i), [ g1==0, 0.1 <= z_2-z{k+1} ]) ];   
% 
% constraints = [constraints, [G1(1,1)+G1(2,1)+G1(3,1)==1], 
%               implies( G1(1,1), [ g1==0, z_2-z{k+1} <=-0.1 ]);
%               implies( G1(2,1), [ g1==1, -0.1<=z_2-z{k+1} <=0.1 ]);
%               implies( G1(3,1), [ g1==0, 0.1 <= z_2-z{k+1} ]) ];   
% 
%           
% constraints = [constraints, 1<=z{k+1}<=L ];           
% 
% %.........................Lateral distance...............................
% 
% constraints = [constraints, [sum(Z1,1,i)==1],... 
%               implies( Z1(1,1,i), [ z1==0,       z_2-p_z <= -1.1 ]);
%               implies( Z1(2,1,i), [ z1==1, -1.1<=z_2-p_z <= -0.9 ]);
%               implies( Z1(3,1,i), [ z1==0, -0.9<=z_2-p_z <= 0.9 ]);
%               implies( Z1(4,1,i), [ z1==1,  0.9<=z_2-p_z <= 1.1 ]);
%               implies( Z1(5,1,i), [ z1==0,            1.1 <= z_2-p_z ]) ];   
% 
% constraints = [constraints, [sum(Z1,1)==1], 
%               implies( Z1(1,1), [ z1==0,       z_2-p_z <= -1.1 ]);
%               implies( Z1(2,1), [ z1==1, -1.1<=z_2-p_z <= -0.9 ]);
%               implies( Z1(3,1), [ z1==0, -0.9<=z_2-p_z <= 0.9 ]);
%               implies( Z1(4,1), [ z1==1,  0.9<=z_2-p_z <= 1.1 ]);
%               implies( Z1(5,1), [ z1==0,            1.1 <= z_2-p_z ]) ];   
% 
% constraints = [constraints, [1<=p_z<=L]];
% 
% %.........................lateral safety distance...............................
% constraints = [constraints, sum(N1,1)==1, 
%               implies( N1(1,1), [ n1==0,       dis12{1} <= -Dl]);
%               implies( N1(2,1), [ n1==1, -Dl<= dis12{1} <= Dl ]);
%               implies( N1(3,1), [ n1==0,  Dl<= dis12{1}]) ];   
%................................................................................
constraints = [constraints,  Aa1{k}*(Bb1{k}*(Ds - dis12{k+1})+(1-Bb1{k})*(Ds + dis12{k+1}))<=0];
% constraints = [constraints,
%               implies(  Aa1{k}==0, [Bb1{k}==0,   dis12{k+1} <= 10000]);
%               implies(  Aa1{k}==0, [Bb1{k}==1,   dis12{k+1} <= 10000]);
%               implies(  Aa1{k}==1, [Bb1{k}==0,   dis12{k+1} <= Ds]);
%               implies(  Aa1{k}==1, [Bb1{k}==1,   dis12{k+1} >= Ds]) ]; 

% constraints = [constraints,  Aa1*Gg1*(-Bb1*(T*(v_2-v{k})+dis12{k})+(1-Bb1)*(T*(v_2-v{k})+dis12{k}))<=0];
% constraints = [constraints, Ss1*Nn1*(z{k}-p_z)==0];
% 


    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
    
end
objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
%% solver definition   

parameters_in = {Vd,Zd,v{1},p_a,p_z,...
                            v_2 , z_2 , dis12{1} , Aa1 , Bb1 };%,Gg1,Ss1,Nn1};
                         
                            
solutions_out = {[a{:}], [z{:}], [v{:}], [a1{:}],  [b1{:}], [dis12{:}] }; %, [g1], [z1], [n1]};
%                                          [a4], [dis15{:}], [b4], [g4], [z4], [n4]};

controller1 = optimizer(constraints, objective,sdpsettings('solver','gurobi'),parameters_in,solutions_out);
%------condiciones iniciales----------
vel=[20; 10; 10];% velociodad inicial
zel=[5; 1; 5]; %carril inicial
% zel2=[zel(2); zel(1); zel(3); zel(4)]; %carril inicial
% zel5=[zel(5); zel(1); zel(2); zel(3); zel(4)]; %carril inicial
Vdes=[30]; %velocidad deseada
Zdes=[1];
%---distancia inicial de cada agente
% disij= Zj-zi
d1i = [100; 0];
d2i = [-d1i(1); -d1i(1)+d1i(2)];% -d1i(1)+d1i(3)];% -d1i(3) + d1i(4)
d3i = [-d1i(2); -d1i(2)+d1i(1)];% -d1i(2)+d1i(3)];

past_a=[0 0 0]';

%define las condiciones iniciales que deben tener las variables
%logicas

 %.....................vehiculo 1..........................
 
 alogic1_1=[zeros(1,N)]; 
 blogic1_1=[ones(1,N)];  
 G1logic_1=[1];     
 S1logic_1=[1]; 
 N1logic_1=[1];   

 
% def_condinicial(zel,  d1i,Dl,alogic1_1,alogic2_1,blogic1_1,blogic2_1,G1logic_1,G2logic_1,S1logic_1,S2logic_1,N1logic_1,N2logic_1);

% hold on
vhist = vel;
zhist = zel;
ahist = past_a;
dhist = d1i;

 % ...his_1orial variables logicas
 a1hist_1=[];
 b1hist_1=[];    
 g1hist_1=[];   
 n1hist_1=[];  

% ..........historial de las predicciones 
 vp1hist=[];  
 zp1hist=[];   
 z1hist_1=[];    

for i = 1:30

%.........................solver vehiculo 1............................
% parameters_in = {Vd,Zd,v{1},p_a,p_z,...
%                             v_2 , z_2 , dis12{1} , Aa1{:} , Bb1{:} };%,Gg1,Ss1,Nn1};        
% solutions_out = {[a{:}], [z{:}], [v{:}], [a1{:}],  [b1{:}]}; %, [g1], [z1], [n1]};


        inputs = {Vdes(1) , Zdes(1) , vel(1) , past_a(1) , zel(1) , ...
                   vel(2) , zel(2) , d1i(1) , alogic1_1 , blogic1_1 }; %, G1logic_1 , S1logic_1 , N1logic_1};
    [solutions1,diagnostics] = controller1{inputs};    
     
    A = solutions1{1};past_a(1) = A(:,1);
    Z = solutions1{2};      zel(1)=Z(:,1);              zp1hist=[zp1hist; Z];
    V = solutions1{3};      vp1hist = [vp1hist; V];
    A1 = solutions1{4};     a1hist_1 = [a1hist_1; A1];   alogic1_1=A1;
    B1 = solutions1{5};     b1hist_1 = [b1hist_1; B1];   blogic1_1=B1;
    DIS = solutions1{6}; 
%     Gg1 = solutions1{6};  g1hist_1=[g1hist_1 Gg1];    G1logic_1=Gg1(:,1);
%     Z1 = solutions1{7};   z1hist_1=[z1hist_1 Z1];     S1logic_1=Z1(:,1);
%     N1 = solutions1{8};   n1hist_1=[n1hist_1 N1];     N1logic_1=N1(:,1);
    
    if diagnostics == 1
        error('you are close, keep trying 1');
    end   

    %------------------graficas--------------------
    %----------------------------------------------
%     subplot(1,2,1);stairs(i:i+length(A)-1,A,'r')
%     subplot(1,2,2);cla;stairs(i:i+N,Z(1,:),'b');hold on;stairs(i:i+N,future_r(1,:),'k')
%     stairs(1:i,xhist(1,:),'g')
    %---------------------------------------------- 
%     d1i = d1i + T * (vel( 2:(size(vel,1)) )-ones( (size(vel,1)-1),1 )*vel(1));
    d1i = d1i+T*(vel(2:(size(vel,1)))-ones((size(vel,1)-1),1)*vel(1));
    
    
    vel = vel+T*past_a;
    vhist = [vhist vel];
    zhist = [zhist zel];
    ahist = [ahist past_a];
    dhist = [dhist d1i];

%     pause(0.05)   

end

vp2hist = [vel(2)*ones(30,N+1)];
zp2hist = [zel(2)*ones(30,N+1)];

vp3hist = [vel(3)*ones(30,N+1)];
zp3hist = [zel(3)*ones(30,N+1)];


Draw_basico(vhist,zhist,...
                            vp1hist,zp1hist,...
                            vp2hist,zp2hist,...
                            vp3hist,zp3hist,...
                                                dhist,T,N)

