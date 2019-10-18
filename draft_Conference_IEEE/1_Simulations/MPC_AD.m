% <<<<<<< HEAD:MPC_AD.m
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
nv=1; %numero de vehiculos
% MPC data
Q = eye(nv);
R = eye(nv);
N = 8;%horizon
T = 0.5; %[s]
Ds=3;%Safety distance
Dl=5.5; %lateral distance
V_max=40;
A_max=7;
L=6;%number of lanes
Mmax=L-1;
mmin=-L+1;
e=1;
Vd = 12;
Zd = 1;
%------------estados deseados-----------
% Zd = sdpvar(repmat(nv,1,1),repmat(1,1,1));%carril deseado
% Vd = sdpvar(repmat(nv,1,1),repmat(1,1,1));%velocidad deseada


% -------------vehiculo i---------------
v = sdpvar(ones(1,N+1)*nv,ones(1,N+1)); %velocidad
v_2 = sdpvar(1); %velocidad
a = sdpvar(ones(1,N+1)*nv,ones(1,N+1)); %aceleracion
dis12 = sdpvar(ones(1,N+1),ones(1,N+1)); %distancia
z = sdpvar(ones(1,N+1)*nv,ones(1,N+1)); %carril
z_2 = sdpvar(1); %carril
dz = sdpvar(ones(1,N),ones(1,N)); %diferencia de carril

a1 = binvar(1,1);
g1 = binvar(1,1);
a2 = binvar(1,1);
Aa = binvar(1,1);
Bb = binvar(1,1);
Gg = binvar(1,1);
Ss = binvar(1,1);
Nn = binvar(1,1);
b1 = binvar(1,1);
z1 = binvar(1,1);
n1 = binvar(1,1);
% a2 = binvar(repmat(1,1,N),repmat(1,1,N));
% a3 = binvar(repmat(1,1,N),repmat(1,1,N));

D1 = binvar(3,1);
G1 = binvar(3,1);
B1 = binvar(2,1);
Z1 = binvar(5,1);
N1 = binvar(3,1);
% D2 = binvar(repmat(1,1,N),repmat(1,1,N));
% D3 = binvar(repmat(1,1,N),repmat(1,1,N));
p_a = sdpvar(1);
p_z = sdpvar(1);
% constraints = [];
constraints = [-0.1 <= diff([p_a a{:}]) <= 0.1];
constraints = [-1 <= diff([p_z z{:}]) <= 1];
objective   = 0;

%-----creacion de funcion objetivo y restricciones--------------------
for k = 1:N
 objective = objective+(v{k}-Vd)'*Q*(v{k}-Vd) + (z{k}-Zd)'*R*(z{k}-Zd); % calculate obj
  
  % Feasible region
    constraints = [constraints ,  1 <= z{k}     <= L,
                                  1 <= z_2      <= L,%tome valores posibles
                                  0<= v{k+1}   <= V_max,%no exceda las velocidades 
                            z{k}-[1]<= z{k+1}   <=z{k}+[1],
                           -A_max   <= a{k}     <= A_max,
                        -10000   <= dis12{k+1} <= 100000,
                           mmin  <= z_2-z{k+1}  <= Mmax];%paso de un carril
                             
    constraints = [constraints, v{k+1} == v{k}+T*a{k}];%velocidad futura
    constraints = [constraints, dis12{k+1} == dis12{k}+T*(v_2-v{k})];
%     constraints = [constraints, dz{k} == z_2-z{k}];
    
% ------------------------------------vehiculo 1-------------------------------    
%------------------si dz=0  -------------------->>>    dij>= Ds----------------
%.........................alpha...............................
constraints = [constraints, [D1(1)+D1(2)+D1(3)==1], 
              implies( D1(1), [ a1==0, z_2-z{k} <=-0.2 ]);
              implies( D1(2), [ a1==1, -0.2<=z_2-z{k} <=0.2 ]);
              implies( D1(3), [ a1==0, 0.2 <= z_2-z{k} ]) ];
%.........................Beta...............................
constraints = [constraints, [sum(B1)==1], 
              implies(B1(1),[ b1==1, dis12{1} >=0 ]);
              implies(B1(2),[ b1==0, dis12{1} <=0 ]) ];
          
constraints = [constraints, [dis12{1}<=100000]]; 
%  
% %.........................Gamma...............................
constraints = [constraints, [G1(1)+G1(2)+G1(3)==1], 
              implies( G1(1), [ g1==0, z_2-z{k+1} <=-0.1 ]);
              implies( G1(2), [ g1==1, -0.1<=z_2-z{k+1} <=0.2 ]);
              implies( G1(3), [ g1==0, 0.1 <= z_2-z{k+1} ]) ];   
          
constraints = [constraints, [1<=z{k+1}<=L]];           

% %.........................Lateral distance...............................
constraints = [constraints, [sum(Z1)==1], 
              implies( Z1(1), [ z1==0,       z_2-p_z <= -1.1 ]);
              implies( Z1(2), [ z1==1, -1.1<=z_2-p_z <= -0.9 ]);
              implies( Z1(3), [ z1==0, -0.9<=z_2-p_z <= 0.9 ]);
              implies( Z1(4), [ z1==1,  0.9<=z_2-p_z <= 1.1 ]);
              implies( Z1(5), [ z1==0,            1.1 <= z_2-p_z ]) ];   
constraints = [constraints, [1<=p_z<=L]];

% %.........................lateral safety distance...............................
constraints = [constraints, [sum(N1)==1], 
              implies( N1(1), [ n1==0,       dis12{1} <= -Dl]);
              implies( N1(2), [ n1==1, -Dl<= dis12{1} <= Dl ]);
              implies( N1(3), [ n1==0,  Dl<= dis12{1}]) ];   




constraints = [constraints,  Aa*(Bb*(Ds - dis12{k+1})+(1-Bb)*(Ds + dis12{k+1}))<=0];

constraints = [constraints,  Aa*Gg*(-Bb*(T*(v_2-v{k})+dis12{k})+(1-Bb)*(T*(v_2-v{k})+dis12{k}))<=0];

constraints = [constraints, Ss*Nn*(z{k}-p_z)>=0];


    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
  
end

objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
%% solver definition   

parameters_in = {v{1},p_a,p_z,v_2,z_2,dis12{1},Aa,Bb,Gg,Ss,Nn};
solutions_out = {[a{:}], [z{:}], [v{:}], [a1], [dis12{:}], [b1], [g1], [z1], [n1]};

controller1 = optimizer(constraints, objective,sdpsettings('solver','cplex'),parameters_in,solutions_out);
%------condiciones iniciales----------
vel=[20; 15];% velociodad inicial
zel=[4; 2]; %carril inicial
Vdes=[15; 20]; %velocidad deseada
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
 z1hist=[];
for i = 1:30
    
%     dz=zel(2)-zel(1);
%   parameters_in = {v{1},p_a,p_z,v_2,z_2,dis12{1},b1};
%   inputs = {past_a(1),vel(1),zel(1),d12,vel(2),zel(2)};
% solutions_out = {[a{:}], [z{:}], [v{:}], [dz{:}], [a1{:}], [b1{:}], [D1{:}]};
    inputs = {vel(1),past_a(1),zel(1),vel(2),zel(2),d12,alogic,blogic,G1logic,S1logic,N1logic};
    [solutions,diagnostics] = controller1{inputs};    
    A = solutions{1};past_a(1) = A(:,1);
    Z = solutions{2}; zel(1)=Z(:,1);zphist=[zphist; Z];
    V = solutions{3};vphist=[vphist; V];
%     DZ = solutions{4};
    A1 = solutions{4};alogic=A1(:,1);
    g1 = solutions{5};    g1hist=[g1hist; g1];
    B1 = solutions{6};    b1hist=[b1hist B1];blogic=B1(:,1);
    Gg1 = solutions{7};    ghist=[ghist Gg1];G1logic=Gg1(:,1);
    Z1 = solutions{8};    z1hist=[z1hist Z1];S1logic=Z1(:,1);
    N1 = solutions{9};    n1hist=[n1hist N1];N1logic=N1(:,1);
    
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
%     D1hist =[D1hist D11];
    A1hist =[A1hist A1];
    d12 = d12+T*(vel(2)-vel(1));
    pause(0.05)   
    % The measured disturbance actually isn't constant, it changes slowly
%     disturbance = 0.99*disturbance + 0.01*randn(1);
end



Draw_MIPG(vhist,vphist,zhist,zphist,dhist,T,N)

