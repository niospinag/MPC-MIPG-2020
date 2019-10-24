
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
nv=2; %numero de vehiculos sin el agente no cooperativo
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
v = sdpvar(ones(1,N+1),ones(1,N+1)); %velocidad del vehiculo actual
a = sdpvar(ones(1,N+1),ones(1,N+1)); %aceleracion actual del vehiculo
z = sdpvar(ones(1,N+1),ones(1,N+1)); %carril actual

v_i = sdpvar(1,nv); %velocidad del otro vehculo
z_i = sdpvar(1,nv); %carril del vehiculo j
dis1i = sdpvar(ones(1,N+1)*nv,ones(1,N+1)); %distancia entre vehiculo 1 y 2

a1 = binvar(1,nv);
g1 = binvar(1,nv);
a2 = binvar(1,nv);
Aa = binvar(1,nv);
Bb = binvar(1,nv);
Gg = binvar(1,nv);
Ss = binvar(1,nv);
Nn = binvar(1,nv);
b1 = binvar(1,nv);
z1 = binvar(1,nv);
n1 = binvar(1,nv);

D1 = binvar(3,1,nv);
G1 = binvar(3,1,nv);
B1 = binvar(2,1,nv);
Z1 = binvar(5,1,nv);
N1 = binvar(3,1,nv);

p_a = sdpvar(1);
p_z = sdpvar(1);

constraints = [-0.1 <= diff([p_a a{:}]) <= 0.1];
constraints = [constraints, -1 <= diff([p_z z{:}]) <= 1];
objective   = 0;
i=1;
%-----creacion de funcion objetivo y restricciones--------------------
for k = 1:N
 objective = objective+(v{k}-Vd)'*Q*(v{k}-Vd) + (z{k}-Zd)'*R*(z{k}-Zd); % calculate obj
  
  % Feasible region
    constraints = [constraints ,1<=    z{k}     <= L,
                               1 <=    z_i      <= L,%tome valores posibles
                                0<=    v{k+1}   <= V_max,%no exceda las velocidades 
                         z{k}-[1]<=    z{k+1}   <=z{k}+[1],
                           -A_max<=    a{k}     <= A_max];%paso de un carril
                             
    constraints = [constraints, v{k+1} == v{k}+T*a{k}];%velocidad futura
  
    for i=1:nv
% ------------------------------------vehiculo 1-------------------------------    
%------------------si dz=0  -------------------->>>    dij>= Ds----------------

    constraints = [constraints, -10000  <=  dis1i{k+1}(i) <= 100000,
                                  mmin  <= z_i(i)-z{k+1}  <= Mmax];
    constraints = [constraints, dis1i{k+1}(i) == dis1i{k}(i)+T*(v_i(i)-v{k})];
%.........................alpha...............................
constraints = [constraints, [D1(1,1,i)+D1(2,1,i)+D1(3,1,i)==1], 
              implies( D1(1,1,i), [ a1(i)==0, z_i(i)-z{k} <=-0.2 ]);
              implies( D1(2,1,i), [ a1(i)==1, -0.2<=z_i(i)-z{k} <=0.2 ]);
              implies( D1(3,1,i), [ a1(i)==0, 0.2 <= z_i(i)-z{k} ]) ];
%.........................Beta...............................
constraints = [constraints, [sum(B1,1,i)==1], 
              implies(B1(1,1,i),[ b1(i)==1, dis1i{1}(i) >=0 ]);
              implies(B1(2,1,i),[ b1(i)==0, dis1i{1}(i) <=0 ]) ];
          
constraints = [constraints, [dis1i{1}(i)<=100000]]; 

% %.........................Gamma...............................
constraints = [constraints, [G1(1,1,i)+G1(2,1,i)+G1(3,1,i)==1], 
              implies( G1(1,1,i), [ g1(i)==0, z_i(i)-z{k+1} <=-0.1 ]);
              implies( G1(2,1,i), [ g1(i)==1, -0.1<=z_i(i)-z{k+1} <=0.2 ]);
              implies( G1(3,1,i), [ g1(i)==0, 0.1 <= z_i(i)-z{k+1} ]) ];   
          
constraints = [constraints, [1<=z{k+1}<=L]];           

% %.........................Lateral distance...............................
constraints = [constraints, [sum(Z1,1,i)==1], 
              implies( Z1(1,1,i), [ z1(i)==0,       z_i(i)-z{k} <= -1.1 ]);
              implies( Z1(2,1,i), [ z1(i)==1, -1.1<=z_i(i)-z{k} <= -0.9 ]);
              implies( Z1(3,1,i), [ z1(i)==0, -0.9<=z_i(i)-z{k} <= 0.9 ]);
              implies( Z1(4,1,i), [ z1(i)==1,  0.9<=z_i(i)-z{k} <= 1.1 ]);
              implies( Z1(5,1,i), [ z1(i)==0,            1.1 <= z_i(i)-z{k} ]) ];   
constraints = [constraints, [1<=p_z<=L]];

% %.........................lateral safety distance...............................
constraints = [constraints, [sum(N1,1,i)==1], 
              implies( N1(1,1,i), [ n1(i)==0,       dis1i{1}(i) <= -Dl]);
              implies( N1(2,1,i), [ n1(i)==1, -Dl<= dis1i{1}(i) <= Dl ]);
              implies( N1(3,1,i), [ n1(i)==0,  Dl<= dis1i{1}(i)]) ];   




constraints = [constraints,  Aa(i)*(Bb(i)*(Ds - dis1i{k+1}(i))+(1-Bb(i))*(Ds + dis1i{k+1}(i)))<=0];

constraints = [constraints,  Aa(i)*Gg(i)*(-Bb(i)*(T*(v_i(i)-v{k})+dis1i{k}(i))+(1-Bb(i))*(T*(v_i(i)-v{k})+dis1i{k}(i)))<=0];


constraints = [constraints, Ss(i)*(Nn(i))*(z{k}-p_z)==0];

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
    end
end
objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
%% solver definition   

parameters_in = {Vd,Zd,v{1},p_a,p_z,v_i,z_i,dis1i{1},Aa,Bb,Gg,Ss,Nn};
solutions_out = {[a{:}], [z{:}], [v{:}], [a1], [dis1i{:}], [b1], [g1], [z1], [n1]};

controller1 = optimizer(constraints, objective,sdpsettings('solver','cplex'),parameters_in,solutions_out);
%------condiciones iniciales----------
vel=[20; 15; 10];% velociodad inicial
zel=[5; 2; 1]; %carril inicial
Vdes=[12; 20; 25]; %velocidad deseada
Zdes=[1; 2; 3];
%---distancia inicial de cada agente
% disij= Zj-zi
d12 = [0; 0];
dis21 = [-40];
past_a=[0 0 0]';


% hold on
vhist = vel;
zhist = zel;
ahist = past_a;
dhist = d12;
 D1hist =[0 0 0]';
 A1hist =[0]';
 alogic=[0];
 blogic=[1];
 G1logic=[1];
 S1logic=[1];
 N1logic=[1];

 n1hist=[];
 g1hist=[];
 vphist=[]; 
zphist=[];
 ghist=[];
 b1hist=[];
 z1hist=[];
for i = 1:30
% parameters_in = {Vd,Zd,v{1},p_a,p_z,v_i,z_i,dis1i{1},Aa,Bb,Gg,Ss,Nn};
% solutions_out = {[a{:}], [z{:}], [v{:}], [a1], [dis1i{:}], [b1], [g1], [z1], [n1]};
    inputs = {Vdes(1),Zdes(1),vel(1),past_a(1),zel(1)            ,vel(2:3),zel(2:3),d12(1:2),alogic,blogic,G1logic,S1logic,N1logic};
    [solutions1,diagnostics] = controller1{inputs};    
    
    A = solutions1{1};past_a(1) = A(:,1);
    Z = solutions1{2};   zel(1)=Z(:,1);          zphist=[zphist; Z];
    V = solutions1{3};   vphist=[vphist; V];
    A1 = solutions1{4};  alogic=A1(:,1);
    g1 = solutions1{5};  g1hist=[g1hist; g1];
    B1 = solutions1{6};  b1hist=[b1hist B1];     blogic=B1(:,1);
    Gg1 = solutions1{7}; ghist=[ghist Gg1];      G1logic=Gg1(:,1);
    Z1 = solutions1{8};  z1hist=[z1hist Z1];     S1logic=Z1(:,1);
    N1 = solutions1{9};  n1hist=[n1hist N1];     N1logic=N1(:,1);
        
    if diagnostics == 1
        error('you are close, keep trying 1');
    end
    
%     % vehiculo 2
%      inputs = {Vdes(3),Zdes(3),vel(3),past_a(3),zel(3)            ,vel(2),zel(2),d12(1),alogic,blogic,G1logic,S1logic,N1logic};
%     [solutions1,diagnostics] = controller1{inputs};    
%     
%     A = solutions1{1};past_a(1) = A(:,1);
%     Z = solutions1{2};   zel(1)=Z(:,1);          zphist=[zphist; Z];
%     V = solutions1{3};   vphist=[vphist; V];
%     A1 = solutions1{4};  alogic=A1(:,1);
%     g1 = solutions1{5};  g1hist=[g1hist; g1];
%     B1 = solutions1{6};  b1hist=[b1hist B1];     blogic=B1(:,1);
%     Gg1 = solutions1{7}; ghist=[ghist Gg1];      G1logic=Gg1(:,1);
%     Z1 = solutions1{8};  z1hist=[z1hist Z1];     S1logic=Z1(:,1);
%     N1 = solutions1{9};  n1hist=[n1hist N1];     N1logic=N1(:,1);
%         
%     if diagnostics == 1
%         error('you are close, keep trying 1');
%     end
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
    d12 = d12+T*(vel(2:(nv+1))-ones(nv,1)*vel(1));
    pause(0.05)   
    % The measured disturbance actually isn't constant, it changes slowly
%     disturbance = 0.99*disturbance + 0.01*randn(1);
end



Draw_MIPG(vhist,vphist,zhist,zphist,dhist,T,N)

