% %UNIVERSIDAD NACIONAL DE COLOMBIA
% Multi Vehicle automated drivring
%Autor: Nestor Ospina
clear all
close all
clc
%----------pc casa



%---------laptop tavo
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
V_max=60;
A_max=3.2;
L=6;%number of lanes
Mmax=L-1;
mmin=-L+1;
e=1;
Vd = 40;
Zd = 1;
%------------estados deseados-----------
% Zd = sdpvar(repmat(nv,1,1),repmat(1,1,1));%carril deseado
% Vd = sdpvar(repmat(nv,1,1),repmat(1,1,1));%velocidad deseada


% -------------vehiculo i---------------
v = sdpvar(repmat(nv,1,N+1),repmat(1,1,N+1));%velocidad
v_2 = sdpvar(1);%velocidad
a = sdpvar(repmat(nv,1,N),repmat(1,1,N));%aceleracion
% dij = sdpvar(repmat(nv,1,N+1),repmat(nv-1,1,N+1));%distancia
dis12 = sdpvar(repmat(1,1,N+1),repmat(1,1,N+1));%distancia
% d21 = sdpvar(repmat(1,1,N+1),repmat(1,1,N+1));%distancia
z = sdpvar(repmat(nv,1,N+1),repmat(1,1,N+1));%carril
z_2 = sdpvar(1);%carril
dz = sdpvar(repmat(1,1,N),repmat(1,1,N));%diferencia de carril


a1 = binvar(1,1);
a2 = binvar(1,1);
b1 = binvar(1,1);
% a2 = binvar(repmat(1,1,N),repmat(1,1,N));
% a3 = binvar(repmat(1,1,N),repmat(1,1,N));

D1 = binvar(3,1);
% D2 = binvar(repmat(1,1,N),repmat(1,1,N));
% D3 = binvar(repmat(1,1,N),repmat(1,1,N));
p_a = sdpvar(1);
p_z = sdpvar(1);
constraints = [];
constraints = [-0.1 <= diff([p_a a{:}]) <= 0.1];
constraints = [-1 <= diff([p_z z{:}]) <= 1];
objective   = 0;

%-----creacion de funcion objetivo y restricciones--------------------
for k = 1:N
 objective = objective+(v{k}-Vd)'*Q*(v{k}-Vd) + (z{k}-Zd)'*R*(z{k}-Zd); % calculate obj
  
  % Feasible region
    constraints = [constraints ,  1 <= z{k}     <= L,
                                  1 <= z_2      <= L,%tome valores posibles
%                                  1<= z{k+1}   <= L,
                                   0<= v{k+1}   <= V_max,%no exceda las velocidades 
%               -max([1 z_1{k}-[1]])<= z_1{k+1} <= min([L z_1{k}+[1]]),
                            z{k}-[1]<= z{k+1}   <=z{k}+[1],
 %                                -1<= z{k+1} - z{k} <= 1,%paso de a un carril
                           -A_max   <= a{k}     <= A_max,
%                          -10000   <= d12{k}+T*(v{k}(2)-v{k}(1)) <= 10000,
                        -10000   <= dis12{k+1} <= 100000,
                           mmin  <= z_2-z{k+1}  <= Mmax];%paso de un carril
                             
    constraints = [constraints, v{k+1} == v{k}+T*a{k}];%velocidad futura
    constraints = [constraints, dis12{k+1} == dis12{k}+T*(v_2-v{k})];
    constraints = [constraints, dz{k} == z_2-z{k}];
    
% ------------------------------------vehiculo 1-------------------------------    
%------------------si dz=0  -------------------->>>    dij>= Ds----------------

constraints = [constraints, [D1(1)+D1(2)+D1(3)==1],
              implies( D1(1), [ a1==0, z_2-z{k} <=-0.2  ]);
              implies( D1(2), [ a1==1, -0.2<=z_2-z{k} <=0.2 ]);
              implies( D1(3), [ a1==0, 0.2 <= z_2-z{k} ])];
                              
% constraints = [constraints,a2==a1];          
constraints = [constraints,  implies( b1, dis12{k+1} >=Ds) ];
%  constraints = [constraints, [sum(b1) == 1], implies(b1(1), [a1==1, dis12{k+1} >=Ds]);
%                                                 implies(b1(2), [a1==0 ] )];
 
%  f(x)???+M(1?a)
%  constraints = [constraints,  implies(a1{k}==1, dis12{k+1} >=Ds)];
%                                             
%                                             
% implies(d1,[a==0,f>=margin]) + 
% implies(d2,[a==1,-margin <= f <= margin]) + 
% implies(d3,[a==0,f <= -margin]) + 
% [d1+d2+d3 == 1]
% % constraints = [constraints, dis12{k+1} >=Ds ];                                    

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
  
end
objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
  

parameters_in = {v{1},p_a,p_z,v_2,z_2,dis12{1},b1};
solutions_out = {[a{:}], [z{:}], [v{:}], [dz{:}], [a1], [D1]};

controller1 = optimizer(constraints, objective,sdpsettings('solver','cplex'),parameters_in,solutions_out);
%------condiciones iniciales----------
vel=[3; 15];% velociodad inicial
zel=[6; 1]; %carril inicial
Vdes=[30; 20]; %velocidad deseada
Zdes=[1; 2];
%---distancia inicial de cada agente
% disij= Zj-zi
d12 = [10];
dis21 = [-40];
past_a=[0 0]';


% clf;

% hold on
vhist = vel;
zhist = zel;
ahist = past_a;
dhist = d12;
 D1hist =[0 0 0]';
 A1hist =[0]';
 blogic=0;
 vphist=[]; zphist=[];
%%
for i = 1:20
    
%     dz=zel(2)-zel(1);
%   parameters_in = {v{1},p_a,p_z,v_2,z_2,dis12{1},b1};
%   inputs = {past_a(1),vel(1),zel(1),d12,vel(2),zel(2)};
% solutions_out = {[a{:}], [z{:}], [v{:}], [dz{:}], [a1{:}], [b1{:}], [D1{:}]};
    inputs = {vel(1),past_a(1),zel(1),vel(2),zel(2),d12,blogic};
    [solutions,diagnostics] = controller1{inputs};    
    A = solutions{1};past_a(1) = A(:,1);
    Z = solutions{2}; zel(1)=Z(:,1);zphist=[zphist; Z];
    V = solutions{3};vphist=[vphist; V];
    DZ = solutions{4};
    A1 = solutions{5};blogic=A1(:,1);
%     B1 = solutions{6};
    D11 = solutions{6};
    
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
    D1hist =[D1hist D11];
    A1hist =[A1hist A1];
    d12 = d12+T*(vel(2)-vel(1));
%     pause(0.05)   
    % The measured disturbance actually isn't constant, it changes slowly
%     disturbance = 0.99*disturbance + 0.01*randn(1);
end



Draw_MIPG(vhist,vphist,zhist,zphist,dhist,T,N)







