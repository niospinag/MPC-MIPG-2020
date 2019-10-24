
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


% %---------laptop tavo
%linux
addpath(genpath('/opt/ibm/ILOG/CPLEX_Studio_Community129/cplex/matlab/x86-64_linux'))%cplex
addpath(genpath('/home/tavocardona/Documents/YALMIP-master/YALMIP-master'))%yalmip
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
N = 8;%horizon
T = 0.5; %[s]
Ds=3;%Safety distance
Dl=15; %lateral distance
V_max=40;
A_max=7;
L=6;%number of lanes
Mmax=L-1;
mmin=-L+1;
e=1;
% Vd = 12;
% Zd = 1;
%------------estados deseados-----------
 Zd = sdpvar(repmat(1,1,1),repmat(1,1,1));%carril deseado
 Vd = sdpvar(repmat(1,1,1),repmat(1,1,1));%velocidad deseada


% -------------vehiculo i---------------
v = sdpvar(ones(1,N+1)*nv,ones(1,N+1)); %velocidad del vehiculo actual
v_2 = sdpvar(1); %velocidad del otro vehculo
a = sdpvar(ones(1,N+1)*nv,ones(1,N+1)); %aceleracion actual del vehiculo
dis1i = sdpvar(ones(1,N+1),ones(1,N+1)); %distancia entre vehiculo 1 y 2
z = sdpvar(ones(1,N+1)*nv,ones(1,N+1)); %carril actual
z_i = sdpvar(1); %carril del vehiculo j

a1 = binvar(1,1);
g1 = binvar(1,1);
a2 = binvar(1,1);
Aa = binvar(1,1);
Bb = binvar(1,1);
Gg = binvar(1,1);
Ss = binvar(1,1);
Nn = binvar(1,1);
b1 = binvar(1,1);
S1 = binvar(1,1);
n1 = binvar(1,1);

D1 = binvar(3,1);
G1 = binvar(3,1);
B1 = binvar(2,1);
S1 = binvar(5,1);
N1 = binvar(3,1);

p_a = sdpvar(1);
p_z = sdpvar(1);

constraints = [-0.1 <= diff([p_a a{:}]) <= 0.1];
constraints = [constraints, -1 <= diff([p_z z{:}]) <= 1];
objective   = 0;

%-----creacion de funcion objetivo y restricciones--------------------
for k = 1:N
 objective = objective+(v{k}-Vd)'*Q*(v{k}-Vd) + (z{k}-Zd)'*R*(z{k}-Zd); % calculate obj
  
  % Feasible region
    constraints = [constraints ,1<=    z{k}     <= L,
                               1 <=    z_i      <= L,%tome valores posibles
                                0<=    v{k+1}   <= V_max,%no exceda las velocidades 
                         z{k}-[1]<=    z{k+1}   <=z{k}+[1],
                           -A_max<=    a{k}     <= A_max,
                         -10000  <=  dis1i{k+1} <= 100000,
                           mmin  <= z_i-z{k+1}  <= Mmax];%paso de un carril
                             
    constraints = [constraints, v{k+1} == v{k}+T*a{k}];%velocidad futura
    constraints = [constraints, dis1i{k+1} == dis1i{k}+T*(v_2-v{k})];
    
% ------------------------------------vehiculo 1-------------------------------    
%------------------si dz=0  -------------------->>>    dij>= Ds----------------

%.........................alpha...............................
constraints = [constraints, [D1(1)+D1(2)+D1(3)==1], 
              implies( D1(1), [ a1==0, z_i-z{k} <=-0.2 ]);
              implies( D1(2), [ a1==1, -0.2<=z_i-z{k} <=0.2 ]);
              implies( D1(3), [ a1==0, 0.2 <= z_i-z{k} ]) ];
%.........................Beta...............................
constraints = [constraints, [sum(B1)==1], 
              implies(B1(1),[ b1==1, dis1i{1} >=0 ]);
              implies(B1(2),[ b1==0, dis1i{1} <=0 ]) ];
          
constraints = [constraints, [dis1i{1}<=100000]]; 

% %.........................Gamma...............................
constraints = [constraints, [G1(1)+G1(2)+G1(3)==1], 
              implies( G1(1), [ g1==0, z_i-z{k+1} <=-0.1 ]);
              implies( G1(2), [ g1==1, -0.1<=z_i-z{k+1} <=0.2 ]);
              implies( G1(3), [ g1==0, 0.1 <= z_i-z{k+1} ]) ];   
          
constraints = [constraints, [1<=z{k+1}<=L]];           

% %.........................Lateral distance...............................
constraints = [constraints, [sum(S1)==1], 
              implies( S1(1), [ S1==0,       z_i-z{k} <= -1.1 ]);
              implies( S1(2), [ S1==1, -1.1<=z_i-z{k} <= -0.9 ]);
              implies( S1(3), [ S1==0, -0.9<=z_i-z{k} <= 0.9 ]);
              implies( S1(4), [ S1==1,  0.9<=z_i-z{k} <= 1.1 ]);
              implies( S1(5), [ S1==0,            1.1 <= z_i-z{k} ]) ];   
constraints = [constraints, [1<=p_z<=L]];

% %.........................lateral safety distance...............................
constraints = [constraints, [sum(N1)==1], 
              implies( N1(1), [ n1==0,       dis1i{1} <= -Dl]);
              implies( N1(2), [ n1==1, -Dl<= dis1i{1} <= Dl ]);
              implies( N1(3), [ n1==0,  Dl<= dis1i{1}]) ];   




constraints = [constraints,  Aa*(Bb*(Ds - dis1i{k+1})+(1-Bb)*(Ds + dis1i{k+1}))<=0];

constraints = [constraints,  Aa*Gg*(-Bb*(T*(v_2-v{k})+dis1i{k})+(1-Bb)*(T*(v_2-v{k})+dis1i{k}))<=0];


constraints = [constraints, Ss*(Nn)*(z{k}-p_z)==0];

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
  
end
objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
%% solver definition   

parameters_in = {Vd,Zd,v{1},p_a,p_z,v_2,z_i,dis1i{1},Aa,Bb,Gg,Ss,Nn};
solutions_out = {[a{:}], [z{:}], [v{:}], [a1], [dis1i{:}], [b1], [g1], [S1], [n1]};

controller1 = optimizer(constraints, objective,sdpsettings('solver','cplex'),parameters_in,solutions_out);
%------condiciones iniciales----------
vel=[20; 15];% velociodad inicial
zel=[5; 2]; %carril inicial
Vdes=[12; 20]; %velocidad deseada
Zdes=[1; 2];
%---distancia inicial de cada agente
% disij= Zj-zi
d12 = [0];
dis21 = [-40];
past_a=[0 0]';


% hold on
vhist = vel;
zhist = zel;
ahist = past_a;
dhist = d12;
 D1hist =[0 0 0]';
 A1hist =[0]';
 alogic=0;
 blogic=1;
 G1logic=1;
 S1logic=1;
 N1logic=1;

 n1hist=[];
 g1hist=[];
 vphist=[]; zphist=[];
 ghist=[];
 b1hist=[];
 S1hist=[];
for i = 1:30
% solutions_out = {[a{:}], [z{:}], [v{:}], [a1], [dis1i{:}], [b1], [g1], [S1], [n1]};
% parameters_in = {Vd,Zd,v{1},p_a,p_z,v_2,z_i,dis1i{1},Aa,Bb,Gg,Ss,Nn};
    inputs = {Vdes(1),Zdes(1),vel(1),past_a(1),zel(1),vel(2),zel(2),d12,alogic,blogic,G1logic,S1logic,N1logic};
    [solutions,diagnostics] = controller1{inputs};    
    A = solutions{1};past_a(1) = A(:,1);
    Z = solutions{2};   zel(1)=Z(:,1);          zphist=[zphist; Z];
    V = solutions{3};   vphist=[vphist; V];
    A1 = solutions{4};  alogic=A1(:,1);
    g1 = solutions{5};  g1hist=[g1hist; g1];
    B1 = solutions{6};  b1hist=[b1hist B1];     blogic=B1(:,1);
    Gg1 = solutions{7}; ghist=[ghist Gg1];      G1logic=Gg1(:,1);
    S1 = solutions{8};  S1hist=[S1hist S1];     S1logic=S1(:,1);
    N1 = solutions{9};  n1hist=[n1hist N1];     N1logic=N1(:,1);
    
    if diagnostics == 1
        error('The problem is infeasible');
    end
    %------------------graficas--------------------
    %----------------------------------------------
%     subplot(1,2,1);stairs(i:i+length(A)-1,A,'r')
%     subplot(1,2,2);cla;stairs(i:i+N,Z(1,:),'b');hold on;stairs(i:i+N,future_r(1,:),'k')
%     stairs(1:i,xhist(1,:),'g')
    %---------------------------------------------- 
    vel = vel+T*past_a;
    vhist = [vhist vel];
    zhist = [zhist zel];
    ahist = [ahist past_a];
    dhist = [dhist d12];
    A1hist =[A1hist A1];
    d12 = d12+T*(vel(2)-vel(1));
    pause(0.05)   
    % The measured disturbance actually isn't constant, it changes slowly
%     disturbance = 0.99*disturbance + 0.01*randn(1);
end



Draw_MIPG(vhist,vphist,zhist,zphist,dhist,T,N)

