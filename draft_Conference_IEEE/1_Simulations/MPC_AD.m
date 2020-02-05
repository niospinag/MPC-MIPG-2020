
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

% %---------laptop tavo
%linux

% addpath(genpath('/home/tavocardona/gurobi811/linux64'))%cplex
% % addpath(genpath('/opt/ibm/ILOG/CPLEX_Studio_Community129/cplex/matlab/x86-64_linux'))%cplex
%  addpath(genpath('/home/tavocardona/Documents/YALMIP-master/YALMIP-master'))%yalmip
% yalmip('clear')

addpath(genpath('/opt/gurobi900/linux64'))%gurobi
% addpath(genpath('/opt/ibm/ILOG/CPLEX_Studio_Community129/cplex/matlab/x86-64_linux'))%cplex
 addpath(genpath('/home/tavocardona/Documents/YALMIP-master'))%yalmip
yalmip('clear')


% %windows
% addpath(genpath('C:\Users\Haptico\Desktop\NESTOR\paths\gurobi')) %Gurobi
% addpath(genpath('C:\Program Files\IBM\ILOG\CPLEX_Studio_Community129\cplex\matlab\x64_win64'))%cplex
% addpath(genpath('C:\Users\Haptico\Desktop\NESTOR\potential games\YALMIP-master'))

%% PROGRAM 
% Model data
nx = 1; % Number of agents
nu = 1; % Number of inputs
nv=1; %numero de vehiculos sin el agente no cooperativo
% MPC data
Q = eye(nv);
R = eye(nv);
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

v_2 = sdpvar(1,nv);     v_3 = sdpvar(1,nv); %velocidad del otro vehculo
v_4 = sdpvar(1,nv);     v_5 = sdpvar(1,nv); %velocidad del otro vehculo
z_2 = sdpvar(1,nv);     z_3 = sdpvar(1,nv); %carril del vehiculo j
z_4 = sdpvar(1,nv);     z_5 = sdpvar(1,nv); %carril del vehiculo j
dis12 = sdpvar(ones(1,N+1)*nv,ones(1,N+1));     dis13 = sdpvar(ones(1,N+1)*nv,ones(1,N+1)); %distancia entre vehiculo 1 y 2
dis14 = sdpvar(ones(1,N+1)*nv,ones(1,N+1));     dis15 = sdpvar(ones(1,N+1)*nv,ones(1,N+1)); %distancia entre vehiculo 1 y 2

a1 = binvar(1,nv);      a2 = binvar(1,nv);      a3 = binvar(1);      a4 = binvar(1,nv);
g1 = binvar(1,nv);      g2 = binvar(1,nv);      g3 = binvar(1,nv);      g4 = binvar(1,nv);
Aa1 = binvar(1,nv);     Aa2 = binvar(1,nv);     Aa3 = binvar(1,nv);     Aa4 = binvar(1,nv);
Bb1 = binvar(1,nv);     Bb2 = binvar(1,nv);     Bb3 = binvar(1,nv);     Bb4 = binvar(1,nv);
Gg1 = binvar(1,nv);     Gg2 = binvar(1,nv);     Gg3 = binvar(1,nv);     Gg4 = binvar(1,nv);
Ss1 = binvar(1,nv);     Ss2 = binvar(1,nv);     Ss3 = binvar(1,nv);     Ss4 = binvar(1,nv);
Nn1 = binvar(1,nv);     Nn2 = binvar(1,nv);     Nn3 = binvar(1,nv);     Nn4 = binvar(1,nv);
b1 = binvar(1,nv);      b2 = binvar(1,nv);      b3 = binvar(1,nv);      b4 = binvar(1,nv);
z1 = binvar(1,nv);      z2 = binvar(1,nv);      z3 = binvar(1,nv);      z4 = binvar(1,nv);
n1 = binvar(1,nv);      n2 = binvar(1,nv);      n3 = binvar(1,nv);      n4 = binvar(1,nv);

D1 = binvar(3,1,nv);    D2 = binvar(3,1,nv);    D3 = binvar(3,1);    D4 = binvar(3,1,nv);
G1 = binvar(3,1,nv);    G2 = binvar(3,1,nv);    G3 = binvar(3,1);    G4 = binvar(3,1,nv);
B1 = binvar(2,1,nv);    B2 = binvar(2,1,nv);    B3 = binvar(2,1);    B4 = binvar(2,1,nv);
Z1 = binvar(5,1,nv);    Z2 = binvar(5,1,nv);    Z3 = binvar(5,1);    Z4 = binvar(5,1,nv);
N1 = binvar(3,1,nv);    N2 = binvar(3,1,nv);    N3 = binvar(3,1);    N4 = binvar(3,1,nv);

p_a = sdpvar(1);
p_z = sdpvar(1);

constraints = -0.8 <= diff([p_a a{:}]) <= 1;
constraints = [constraints, -1 <= diff([p_z z{:}]) <= 1];
objective   = 0;

%-----creacion de funcion objetivo y restricciones--------------------
for k = 1:N
 objective = objective+(v{k}-Vd)'*Q*(v{k}-Vd) + (z{k}-Zd)'*R*(z{k}-Zd); % calculate obj
  
  % Feasible region
    constraints = [constraints,1 <=    z{k}     <= L,
                               1 <=    z_2      <= L,%tome valores posibles
                               1 <=    z_3      <= L,%tome valores posibles
                               1 <=    z_4      <= L,%tome valores posibles
                               1 <=    z_5      <= L,%tome valores posibles
                                0<=    v{k+1}   <= V_max,%no exceda las velocidades 
                         z{k}-[1]<=    z{k+1}   <=z{k}+[1],
                           -A_max<=    a{k}     <= A_max];%paso de un carril
                             
    constraints = [constraints, -1 <= diff([z{k+1} z{k}]) <= 1];
    
    constraints = [constraints, v{k+1} == v{k}+T*a{k}];%velocidad futura
  
   
% ------------------------------------vehiculo 2-------------------------------    
%------------------si dz=0  -------------------->>>    dij>= Ds----------------

    constraints = [constraints, -10000  <=  dis12{k+1} <= 100000,...
                                  mmin  <= z_2-z{k+1}  <= Mmax];
    constraints = [constraints, dis12{k+1} == dis12{k}+T*(v_2-v{k})];
%.........................alpha...............................

constraints = [constraints, [D1(1,1,i)+D1(2,1,i)+D1(3,1,i)==1],... 
              implies( D1(1,1,i), [ a1==0, z_2-z{k} <=-0.1 ]);
              implies( D1(2,1,i), [ a1==1, -0.1<=z_2-z{k} <=0.1 ]);
              implies( D1(3,1,i), [ a1==0, 0.1 <= z_2-z{k} ]) ];
%.........................Beta...............................
constraints = [constraints, [sum(B1,1,i)==1],... 
              implies(B1(1,1,i),[ b1==1, dis12{1} >=0 ]);
              implies(B1(2,1,i),[ b1==0, dis12{1} <=0 ]) ];

constraints = [constraints, [D1(1,1)+D1(2,1)+D1(3,1)==1], 
              implies( D1(1,1), [ a1==0, z_2-z{k} <=-0.1 ]);
              implies( D1(2,1), [ a1==1, -0.1<=z_2-z{k} <=0.1 ]);
              implies( D1(3,1), [ a1==0, 0.1 <= z_2-z{k} ]) ];
%.........................Beta...............................
constraints = [constraints, [sum(B1,1)==1], 
              implies(B1(1,1),[ b1==1, dis12{1} >=0 ]);
              implies(B1(2,1),[ b1==0, dis12{1} <=0 ]) ];

          
constraints = [constraints, [dis12{1}<=100000]]; 

% %.........................Gamma...............................

constraints = [constraints, [G1(1,1,i)+G1(2,1,i)+G1(3,1,i)==1],... 
              implies( G1(1,1,i), [ g1==0, z_2-z{k+1} <=-0.1 ]);
              implies( G1(2,1,i), [ g1==1, -0.1<=z_2-z{k+1} <=0.1 ]);
              implies( G1(3,1,i), [ g1==0, 0.1 <= z_2-z{k+1} ]) ];   

constraints = [constraints, [G1(1,1)+G1(2,1)+G1(3,1)==1], 
              implies( G1(1,1), [ g1==0, z_2-z{k+1} <=-0.1 ]);
              implies( G1(2,1), [ g1==1, -0.1<=z_2-z{k+1} <=0.1 ]);
              implies( G1(3,1), [ g1==0, 0.1 <= z_2-z{k+1} ]) ];   

          
constraints = [constraints, 1<=z{k+1}<=L ];           

% %.........................Lateral distance...............................

constraints = [constraints, [sum(Z1,1,i)==1],... 
              implies( Z1(1,1,i), [ z1==0,       z_2-p_z <= -1.1 ]);
              implies( Z1(2,1,i), [ z1==1, -1.1<=z_2-p_z <= -0.9 ]);
              implies( Z1(3,1,i), [ z1==0, -0.9<=z_2-p_z <= 0.9 ]);
              implies( Z1(4,1,i), [ z1==1,  0.9<=z_2-p_z <= 1.1 ]);
              implies( Z1(5,1,i), [ z1==0,            1.1 <= z_2-p_z ]) ];   

constraints = [constraints, [sum(Z1,1)==1], 
              implies( Z1(1,1), [ z1==0,       z_2-p_z <= -1.1 ]);
              implies( Z1(2,1), [ z1==1, -1.1<=z_2-p_z <= -0.9 ]);
              implies( Z1(3,1), [ z1==0, -0.9<=z_2-p_z <= 0.9 ]);
              implies( Z1(4,1), [ z1==1,  0.9<=z_2-p_z <= 1.1 ]);
              implies( Z1(5,1), [ z1==0,            1.1 <= z_2-p_z ]) ];   

constraints = [constraints, [1<=p_z<=L]];

% %.........................lateral safety distance...............................
constraints = [constraints, sum(N1,1)==1, 
              implies( N1(1,1), [ n1==0,       dis12{1} <= -Dl]);
              implies( N1(2,1), [ n1==1, -Dl<= dis12{1} <= Dl ]);
              implies( N1(3,1), [ n1==0,  Dl<= dis12{1}]) ];   
%................................................................................
constraints = [constraints,  Aa1*(Bb1*(Ds - dis12{k+1})+(1-Bb1)*(Ds + dis12{k+1}))<=0];
constraints = [constraints,  Aa1*Gg1*(-Bb1*(T*(v_2-v{k})+dis12{k})+(1-Bb1)*(T*(v_2-v{k})+dis12{k}))<=0];
constraints = [constraints, Ss1*Nn1*(z{k}-p_z)==0];

% ------------------------------------vehiculo 3-------------------------------    
%------------------si dz=0  -------------------->>>    dij>= Ds----------------

    constraints = [constraints, -10000  <=  dis13{k+1} <= 100000,
                                  mmin  <= z_3-z{k+1}  <= Mmax];
    constraints = [constraints, dis13{k+1} == dis13{k}+T*(v_3-v{k})];
%.........................alpha...............................
constraints = [constraints, [D2(1,1)+D2(2,1)+D2(3,1)==1], 
              implies( D2(1,1), [ a2==0, z_3-z{k} <=-0.1 ]);
              implies( D2(2,1), [ a2==1, -0.1<=z_3-z{k} <=0.1 ]);
              implies( D2(3,1), [ a2==0, 0.1 <= z_3-z{k} ]) ];
%.........................Beta...............................
constraints = [constraints, [sum(B2,1)==1], 
              implies(B2(1,1),[ b2==1, dis13{1} >=0 ]);
              implies(B2(2,1),[ b2==0, dis13{1} <=0 ]) ];
          
constraints = [constraints, [dis13{1}<=100000]]; 

% %.........................Gamma...............................
constraints = [constraints, [G2(1,1)+G2(2,1)+G2(3,1)==1], 
              implies( G2(1,1), [ g2==0, z_3-z{k+1} <=-0.1 ]);
              implies( G2(2,1), [ g2==1, -0.1<=z_3-z{k+1} <=0.1 ]);
              implies( G2(3,1), [ g2==0, 0.1 <= z_3-z{k+1} ]) ];   
          
constraints = [constraints, [1<=z{k+1}<=L]];           

% %.........................Lateral distance...............................
constraints = [constraints, [sum(Z2,1)==1], 
              implies( Z2(1,1), [ z2==0,       z_3-p_z <= -1.1 ]);
              implies( Z2(2,1), [ z2==1, -1.1<=z_3-p_z <= -0.9 ]);
              implies( Z2(3,1), [ z2==0, -0.9<=z_3-p_z <= 0.9 ]);
              implies( Z2(4,1), [ z2==1,  0.9<=z_3-p_z <= 1.1 ]);
              implies( Z2(5,1), [ z2==0,            1.1 <= z_3-p_z ]) ];   
constraints = [constraints, [1<=p_z<=L]];

% %.........................lateral safety distance...............................
constraints = [constraints, [sum(N2,1)==1], 
              implies( N2(1,1), [ n2==0,       dis13{1} <= -Dl]);
              implies( N2(2,1), [ n2==1, -Dl<= dis13{1} <= Dl ]);
              implies( N2(3,1), [ n2==0,  Dl<= dis13{1}]) ];   
%................................................................................
constraints = [constraints,  Aa2*(Bb2*(Ds - dis13{k+1})+(1-Bb2)*(Ds + dis13{k+1}))<=0];
constraints = [constraints,  Aa2*Gg2*(-Bb2*(T*(v_3-v{k})+dis13{k})+(1-Bb2)*(T*(v_3-v{k})+dis13{k}))<=0];
constraints = [constraints, Ss2*(Nn2)*(z{k}-p_z)==0];


% ------------------------------------vehiculo 4-------------------------------    
%------------------si dz=0  -------------------->>>    dij>= Ds----------------

    constraints = [constraints, -10000  <=  dis14{k+1} <= 100000,
                                  mmin  <= z_4-z{k+1}  <= Mmax];
    constraints = [constraints, dis14{k+1} == dis14{k}+T*(v_4-v{k})];
%.........................alpha...............................
constraints = [constraints, [D3(1,1)+D3(2,1)+D3(3,1)==1], 
              implies( D3(1,1), [ a3==0, z_4-z{k} <=-0.1 ]);
              implies( D3(2,1), [ a3==1, -0.1<=z_4-z{k} <=0.1 ]);
              implies( D3(3,1), [ a3==0, 0.1 <= z_4-z{k} ]) ];
%.........................Beta...............................
constraints = [constraints, [sum(B3,1)==1], 
              implies(B3(1,1),[ b3==1, dis14{1} >=0 ]);
              implies(B3(2,1),[ b3==0, dis14{1} <=0 ]) ];
          
constraints = [constraints, [dis14{1}<=100000]]; 

% %.........................Gamma...............................
constraints = [constraints, [G3(1,1)+G3(2,1)+G3(3,1)==1],
              implies( G3(1,1), [ g3==0, z_4-z{k+1} <=-0.1 ]);
              implies( G3(2,1), [ g3==1, -0.1<=z_4-z{k+1} <=0.1 ]);
              implies( G3(3,1), [ g3==0, 0.1 <= z_4-z{k+1} ]) ];   
          
constraints = [constraints, [1<=z{k+1}<=L]];           

% %.........................Lateral distance...............................
constraints = [constraints, [sum(Z3,1)==1], 
              implies( Z3(1,1), [ z3==0,       z_4-z{k} <= -1.1 ]);
              implies( Z3(2,1), [ z3==1, -1.1<=z_4-z{k} <= -0.9 ]);
              implies( Z3(3,1), [ z3==0, -0.9<=z_4-z{k} <= 0.9 ]);
              implies( Z3(4,1), [ z3==1,  0.9<=z_4-z{k} <= 1.1 ]);
              implies( Z3(5,1), [ z3==0,            1.1 <= z_4-z{k} ]) ];   
constraints = [constraints, [1<=p_z<=L]];

% %.........................lateral safety distance...............................
constraints = [constraints, [sum(N3,1)==1], 
              implies( N3(1,1), [ n3==0,       dis14{1} <= -Dl]);
              implies( N3(2,1), [ n3==1, -Dl<= dis14{1} <= Dl ]);
              implies( N3(3,1), [ n3==0,  Dl<= dis14{1}]) ];   
%................................................................................
constraints = [constraints,  Aa3*(Bb3*(Ds - dis14{k+1})+(1-Bb3)*(Ds + dis14{k+1}))<=0];
constraints = [constraints,  Aa3*Gg3*(-Bb3*(T*(v_4-v{k})+dis14{k})+(1-Bb3)*(T*(v_4-v{k})+dis14{k}))<=0];
constraints = [constraints, Ss3*(Nn3)*(z{k}-p_z)==0];


    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
    
end
objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
%% solver definition   

parameters_in = {Vd,Zd,v{1},p_a,p_z,...
                            v_2,z_2,dis12{1},Aa1,Bb1,Gg1,Ss1,Nn1,...
                            v_3,z_3,dis13{1},Aa2,Bb2,Gg2,Ss2,Nn2,...
                            v_4,z_4,dis14{1},Aa3,Bb3,Gg3,Ss3,Nn3};
%                             v_5,z_5,dis15{1},Aa4,Bb4,Gg4,Ss4,Nn4};
                            
solutions_out = {[a{:}], [z{:}], [v{:}], [a1],  [b1], [g1], [z1], [n1]...
                                         [a2],  [b2], [g2], [z2], [n2]...
                                         [a3],  [b3], [g3], [z3], [n3]};
%                                          [a4], [dis15{:}], [b4], [g4], [z4], [n4]};

controller1 = optimizer(constraints, objective,sdpsettings('solver','gurobi'),parameters_in,solutions_out);
%------condiciones iniciales----------
vel=[20; 25; 10; 20];% velociodad inicial
zel=[2; 4; 1; 5]; %carril inicial
zel2=[zel(2); zel(1); zel(3); zel(4)]; %carril inicial
zel3=[zel(3); zel(1); zel(2); zel(4)]; %carril inicial
zel4=[zel(4); zel(1); zel(2); zel(3)]; %carril inicial
% zel5=[zel(5); zel(1); zel(2); zel(3); zel(4)]; %carril inicial
Vdes=[10; 35; 35; 15]; %velocidad deseada
Zdes=[5; 2; 3; 4];
%---distancia inicial de cada agente
% disij= Zj-zi
d1i = [20; 0; 20];
d2i = [-d1i(1); -d1i(1)+d1i(2); -d1i(1)+d1i(3)];
d3i = [-d1i(2); -d1i(2)+d1i(1); -d1i(2)+d1i(3)];
d4i = [-d1i(3); -d1i(3)+d1i(1); -d1i(3)+d1i(2)];%; -d1i(3)+d1i(4)
past_a=[0 0 0 0]';

%define las condiciones iniciales que deben tener las variables
%logicas

 %.....................vehiculo 1.....................................................vehiculo 2
 
 alogic1_1=[0]; alogic2_1=[0];alogic3_1=[0]; alogic4_1=[0];     alogic1_2=[0]; alogic2_2=[0];alogic3_2=[0]; alogic4_2=[0];
 blogic1_1=[1]; blogic2_1=[0];blogic3_1=[1]; blogic4_1=[0];     blogic1_2=[1]; blogic2_2=[0];blogic3_2=[1]; blogic4_2=[0];
 G1logic_1=[1]; G2logic_1=[1];G3logic_1=[1]; G4logic_1=[1];     G1logic_2=[1]; G2logic_2=[1];G3logic_2=[1]; G4logic_2=[1];
 S1logic_1=[1]; S2logic_1=[1];S3logic_1=[1]; S4logic_1=[1];     S1logic_2=[1]; S2logic_2=[1];S3logic_2=[1]; S4logic_2=[1];
 N1logic_1=[1]; N2logic_1=[1];N3logic_1=[1]; N4logic_1=[1];     N1logic_2=[1]; N2logic_2=[1];N3logic_2=[1]; N4logic_2=[1];

 %.....................vehiculo 3.....................................................vehiculo 4
 
 alogic1_3=[0]; alogic2_3=[0];alogic3_3=[0]; alogic4_3=[0];     alogic1_4=[0]; alogic2_4=[0];alogic3_4=[0]; alogic4_4=[0];
 blogic1_3=[1]; blogic2_3=[0];blogic3_3=[1]; blogic4_3=[0];     blogic1_4=[1]; blogic2_4=[0];blogic3_4=[1]; blogic4_4=[0];
 G1logic_3=[1]; G2logic_3=[1];G3logic_3=[1]; G4logic_3=[1];     G1logic_4=[1]; G2logic_4=[1];G3logic_4=[1]; G4logic_4=[1];
 S1logic_3=[1]; S2logic_3=[1];S3logic_3=[1]; S4logic_3=[1];     S1logic_4=[1]; S2logic_4=[1];S3logic_4=[1]; S4logic_4=[1];
 N1logic_3=[1]; N2logic_3=[1];N3logic_3=[1]; N4logic_3=[1];     N1logic_4=[1]; N2logic_4=[1];N3logic_4=[1]; N4logic_4=[1];

 
def_condinicial(zel,  d1i,Dl,alogic1_1,alogic2_1,blogic1_1,blogic2_1,G1logic_1,G2logic_1,S1logic_1,S2logic_1,N1logic_1,N2logic_1);
def_condinicial(zel2, d2i,Dl,alogic1_2,alogic2_2,blogic1_2,blogic2_2,G1logic_2,G2logic_2,S1logic_2,S2logic_2,N1logic_2,N2logic_2);
def_condinicial(zel3, d3i,Dl,alogic1_3,alogic2_3,blogic1_3,blogic2_3,G1logic_3,G2logic_3,S1logic_3,S2logic_3,N1logic_3,N2logic_3);
def_condinicial(zel4, d4i,Dl,alogic1_4,alogic2_4,blogic1_4,blogic2_4,G1logic_4,G2logic_4,S1logic_4,S2logic_4,N1logic_4,N2logic_4);


% hold on
vhist = vel;
zhist = zel;
ahist = past_a;
dhist = d1i;


rastreo=[];
rastreon=[];
%
% ...historial variables logicas
 A1hist =[0]';  A2hist =[0]';   A3hist =[0]';   A4hist =[0]';
 b1hist=[];     b2hist=[];      b3hist=[];      b4hist=[];
 g1hist=[];     g2hist=[];      g3hist=[];      g4hist=[];
 n1hist=[];     n2hist=[];      n3hist=[];      n4hist=[];
 % ...his_1orial variables logicas
 A1hist_1 =[0]';  A2hist_1 =[0]';   A3hist_1 =[0]';   A4hist_1 =[0]';
 b1hist_1=[];     b2hist_1=[];      b3hist_1=[];      b4hist_1=[];
 g1hist_1=[];     g2hist_1=[];      g3hist_1=[];      g4hist_1=[];
 n1hist_1=[];     n2hist_1=[];      n3hist_1=[];      n4hist_1=[];
 % ...his_2orial variables logicas
 A1hist_2 =[0]';  A2hist_2 =[0]';   A3hist_2 =[0]';   A4hist_2 =[0]';
 b1hist_2=[];     b2hist_2=[];      b3hist_2=[];      b4hist_2=[];
 g1hist_2=[];     g2hist_2=[];      g3hist_2=[];      g4hist_2=[];
 n1hist_2=[];     n2hist_2=[];      n3hist_2=[];      n4hist_2=[];
 % ...his_3orial variables logicas
 A1hist_3 =[0]';  A2hist_3 =[0]';   A3hist_3 =[0]';   A4hist_3 =[0]';
 b1hist_3=[];     b2hist_3=[];      b3hist_3=[];      b4hist_3=[];
 g1hist_3=[];     g2hist_3=[];      g3hist_3=[];      g4hist_3=[];
 n1hist_3=[];     n2hist_3=[];      n3hist_3=[];      n4hist_3=[];
% ..........historial de las predicciones 
 vp1hist=[];    vp2hist=[];     vp3hist=[];     vp4hist=[]; 
 zp1hist=[];    zp2hist=[];     zp3hist=[];     zp4hist=[];
 z1hist=[];     z2hist=[];      z3hist=[];      z4hist=[];
 z1hist_1=[];     z2hist_1=[];      z3hist_1=[];      z4hist_1=[];
 z3hist_3=[];
 n3hist_3=[];
for i = 1:30
% parameters_in = {Vd,Zd,v{1},p_a,p_z,...
%                             v_2,z_2,dis12{1},Aa1,Bb1,Gg1,Ss1,Nn1,...
%                             v_3,z_3,dis13{1},Aa2,Bb2,Gg2,Ss2,Nn2,...
%                             v_4,z_4,dis14{1},Aa3,Bb3,Gg3,Ss3,Nn3};
% %                             v_5,z_5,dis15{1},Aa4,Bb4,Gg4,Ss4,Nn4};
%                             
% solutions_out = {[a{:}], [z{:}], [v{:}], [a1],  [b1], [g1], [z1], [n1]...
%                                          [a2],  [b2], [g2], [z2], [n2]...
%                                          [a3],  [b3], [g3], [z3], [n3]};

% 
%      %.........................solver vehiculo 4............................
% inputs4 = {Vdes(4),Zdes(4),vel(4),past_a(4),zel(4),...
%                         vel(1),zel(1),d4i(1),alogic1_4,blogic1_4,G1logic_4,S1logic_4,N1logic_4,...
%                         vel(2),zel(2),d4i(2),alogic2_4,blogic2_4,G2logic_4,S2logic_4,N2logic_4,...
%                         vel(3),zel(3),d4i(3),alogic3_4,blogic3_4,G3logic_4,S3logic_4,N3logic_4};
%     [solutions4,diagnostics4] = controller1{inputs4};           
% 
%  
%     
%     A = solutions4{1};      past_a(4) = A(:,1);
%     Z = solutions4{2};      zel(4)=Z(:,1);          zp4hist=[zp4hist; Z];
%     V = solutions4{3};      vp4hist=[vp4hist; V];
%     A1 = solutions4{4};                             alogic1_4=A1(:,1);
%     B1 = solutions4{5};     b1hist=[b1hist B1];     blogic1_4=B1(:,1);
%     Gg1 = solutions4{6};    g1hist=[g1hist Gg1];    G1logic_4=Gg1(:,1);
%     Z1 = solutions4{7};     z1hist=[z1hist Z1];     S1logic_4=Z1(:,1);
%     N1 = solutions4{8};     n1hist=[n1hist N1];     N1logic_4=N1(:,1);
%     
%     A2 = solutions4{9};                             alogic2_4=A2(:,1);
%     B2 = solutions4{10};    b2hist=[b2hist B2];     blogic2_4=B2(:,1);
%     Gg2 = solutions4{11};   g2hist=[g2hist Gg2];    G2logic_4=Gg2(:,1);
%     Z2 = solutions4{12};    z2hist=[z2hist Z2];     S2logic_4=Z2(:,1);
%     N2 = solutions4{13};    n2hist=[n2hist N2];     N2logic_4=N2(:,1); 
%    
%     A3 = solutions4{14};                            alogic3_4=A3(:,1);
%     B3 = solutions4{15};    b3hist=[b3hist B3];     blogic3_4=B3(:,1);
%     Gg3 = solutions4{16};   g3hist=[g3hist Gg3];    G3logic_4=Gg3(:,1);
%     Z3 = solutions4{17};    z3hist=[z3hist Z3];     S3logic_4=Z3(:,1);
%     N3 = solutions4{18};    n3hist=[n3hist N3];     N3logic_4=N3(:,1);
%     
%     if diagnostics4 == 1
%         error('you are close, keep trying 4');
%     end  



%.........................solver vehiculo 1............................

        inputs = {Vdes(1),Zdes(1),vel(1),past_a(1),zel(1),...
                        vel(2),zel(2),d1i(1),alogic1_1,blogic1_1,G1logic_1,S1logic_1,N1logic_1,...
                        vel(3),zel(3),d1i(2),alogic2_1,blogic2_1,G2logic_1,S2logic_1,N2logic_1,...
                        vel(4),zel(4),d1i(3),alogic3_1,blogic3_1,G3logic_1,S3logic_1,N3logic_1};
    [solutions1,diagnostics] = controller1{inputs};    
     
    A = solutions1{1};past_a(1) = A(:,1);
    Z = solutions1{2};   zel(1)=Z(:,1);             zp1hist=[zp1hist; Z];
    V = solutions1{3};   vp1hist=[vp1hist; V];
    A1 = solutions1{4};                             alogic1_1=A1(:,1);
    B1 = solutions1{5};  b1hist_1=[b1hist_1 B1];    blogic1_1=B1(:,1);
    Gg1 = solutions1{6}; g1hist_1=[g1hist_1 Gg1];   G1logic_1=Gg1(:,1);
    Z1 = solutions1{7};  z1hist_1=[z1hist_1 Z1];    S1logic_1=Z1(:,1);
    N1 = solutions1{8};  n1hist_1=[n1hist_1 N1];    N1logic_1=N1(:,1);
    
    A2 = solutions1{9};                             alogic2_1=A2(:,1);
    B2 = solutions1{10};  b2hist_1=[b2hist_1 B2];   blogic2_1=B2(:,1);
    Gg2 = solutions1{11}; g2hist_1=[g2hist_1 Gg2];  G2logic_1=Gg2(:,1);
    Z2 = solutions1{12};  z2hist_1=[z2hist_1 Z2];   S2logic_1=Z2(:,1);
    N2 = solutions1{13};  n2hist_1=[n2hist_1 N2];   N2logic_1=N2(:,1);    
    
    A3 = solutions1{14};                            alogic3_1=A3(:,1);
    B3 = solutions1{15};  b3hist_1=[b3hist_1 B3];   blogic3_1=B3(:,1);
    Gg3 = solutions1{16}; g3hist_1=[g3hist_1 Gg3];  G3logic_1=Gg3(:,1);
    Z3 = solutions1{17};  z3hist_1=[z3hist_1 Z3];   S3logic_1=Z3(:,1);
    N3 = solutions1{18};  n3hist_1=[n3hist_1 N3];   N3logic_1=N3(:,1);
    
    
%     estar pendiente de s2logic_1, ahi esta el problema

        
    if diagnostics == 1
        error('you are close, keep trying 1');
    end   
    



%.........................solver vehiculo 2............................
inputs2 = {Vdes(2),Zdes(2),vel(2),past_a(2),zel(2),...
                        vel(1),zel(1),d2i(1),alogic1_2,blogic1_2,G1logic_2,S1logic_2,N1logic_2,...
                        vel(3),zel(3),d2i(2),alogic2_2,blogic2_2,G2logic_2,S2logic_2,N2logic_2,...
                        vel(4),zel(4),d2i(3),alogic3_2,blogic3_2,G3logic_2,S3logic_2,N3logic_2};
    [solutions2,diagnostics2] = controller1{inputs2};           

 
    
    A = solutions2{1};past_a(2) = A(:,1);
    Z = solutions2{2};   zel(2)=Z(:,1);          zp2hist=[zp2hist; Z];
    V = solutions2{3};   vp2hist=[vp2hist; V];
    A1 = solutions2{4};                         alogic1_2=A1(:,1);
    B1 = solutions2{5};  b1hist_1=[b1hist B1];  blogic1_2=B1(:,1);
    Gg1 = solutions2{6}; g1hist_1=[g1hist Gg1]; G1logic_2=Gg1(:,1);
    Z1 = solutions2{7};  z1hist=[z1hist Z1];    S1logic_2=Z1(:,1);
    N1 = solutions2{8};  n1hist=[n1hist N1];    N1logic_2=N1(:,1);
    
    A2 = solutions2{9};                        alogic2_2=A2(:,1);
    B2 = solutions2{10};  b2hist=[b2hist B2];   blogic2_2=B2(:,1);
    Gg2 = solutions2{11}; g2hist=[g2hist Gg2];  G2logic_2=Gg2(:,1);
    Z2 = solutions2{12};  z2hist=[z2hist Z2];   S2logic_2=Z2(:,1);
    N2 = solutions2{13};  n2hist=[n2hist N2];   N2logic_2=N2(:,1); 
   
    A3 = solutions2{14};                          alogic3_2=A3(:,1);
    B3 = solutions2{15};  b3hist=[b3hist B3];     blogic3_2=B3(:,1);
    Gg3 = solutions2{16}; g3hist=[g3hist Gg3];    G3logic_2=Gg3(:,1);
    Z3 = solutions2{17};  z3hist=[z3hist Z3];     S3logic_2=Z3(:,1);
    N3 = solutions2{18};  n3hist=[n3hist N3];     N3logic_2=N3(:,1);

    if diagnostics2 == 1
        error('you are close, keep trying 2');
    end  
    
     %.........................solver vehiculo 3............................
inputs3 = {Vdes(3),Zdes(3),vel(3),past_a(3),zel(3),...
                        vel(1),zel(1),d3i(1),alogic1_3,blogic1_3,G1logic_3,S1logic_3,N1logic_3,...
                        vel(2),zel(2),d3i(2),alogic2_3,blogic2_3,G2logic_3,S2logic_3,N2logic_3,...
                        vel(4),zel(4),d3i(3),alogic3_3,blogic3_3,G3logic_3,S3logic_3,N3logic_3};
    [solutions3,diagnostics3] = controller1{inputs3};           

 
    
    A = solutions3{1};      past_a(3) = A(:,1);
    Z = solutions3{2};      zel(3)=Z(:,1);          zp3hist=[zp3hist; Z];
    V = solutions3{3};      vp3hist=[vp3hist; V];
    A1 = solutions3{4};                             alogic1_3=A1(:,1);
    B1 = solutions3{5};     b1hist_3=[b1hist B1];   blogic1_3=B1(:,1);
    Gg1 = solutions3{6};    g1hist_3=[g1hist Gg1];  G1logic_3=Gg1(:,1);
    Z1 = solutions3{7};     z1hist=[z1hist Z1];     S1logic_3=Z1(:,1);
    N1 = solutions3{8};     n1hist=[n1hist N1];     N1logic_3=N1(:,1);
    
    A2 = solutions3{9};                             alogic2_3=A2(:,1);
    B2 = solutions3{10};    b2hist=[b2hist B2];     blogic2_3=B2(:,1);
    Gg2 = solutions3{11};   g2hist=[g2hist Gg2];    G2logic_3=Gg2(:,1);
    Z2 = solutions3{12};    z2hist=[z2hist Z2];     S2logic_3=Z2(:,1);
    N2 = solutions3{13};    n2hist=[n2hist N2];     N2logic_3=N2(:,1); 
   
    A3 = solutions3{14};                            alogic3_3=A3(:,1);
    B3 = solutions3{15};    b3hist=[b3hist B3];     blogic3_3=B3(:,1);
    Gg3 = solutions3{16};   g3hist=[g3hist Gg3];    G3logic_3=Gg3(:,1);
    Z3 = solutions3{17};    z3hist_3=[z3hist_3 Z3]; S3logic_3=Z3(:,1);
    N3 = solutions3{18};    n3hist_3=[n3hist_3 N3]; N3logic_3=N3(:,1);
    
    if diagnostics3 == 1
        error('you are close, keep trying 3');
    end  
       
    
    
    
    
    
    
    %------------------graficas--------------------
    %----------------------------------------------
%     subplot(1,2,1);stairs(i:i+length(A)-1,A,'r')
%     subplot(1,2,2);cla;stairs(i:i+N,Z(1,:),'b');hold on;stairs(i:i+N,future_r(1,:),'k')
%     stairs(1:i,xhist(1,:),'g')
    %---------------------------------------------- 
    d1i = d1i+T*(vel(2:(size(vel,1)))-ones((size(vel,1)-1),1)*vel(1));
    d2i = [-d1i(1); -d1i(1)+d1i(2); -d1i(1)+d1i(3)];
    d3i = [-d1i(2); -d1i(2)+d1i(1); -d1i(2)+d1i(3)];
    d4i = [-d1i(3); -d1i(3)+d1i(1); -d1i(3)+d1i(2)];
    
    
    vel = vel+T*past_a;
    vhist = [vhist vel];
    zhist = [zhist zel];
    ahist = [ahist past_a];
    dhist = [dhist d1i];
%     dhist4 = [dhist4 d4i];

%     A2hist =[A2hist A2];
  
%     pause(0.05)   

end



Draw_MIPG(vhist,zhist,...
                            vp1hist,zp1hist,...
                            vp2hist,zp2hist,...
                            vp3hist,zp3hist,...
                                                dhist,T,N)

