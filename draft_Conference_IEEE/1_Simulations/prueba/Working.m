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
nv=2; %numero de vehiculos sin el agente no cooperativo
% MPC data
Q = 1*eye(1);
R = 10*eye(1);
N = 3;%horizon
T = 0.3; %[s]
Ds=15;%Safety distance [m]
Dl=25; %lateral distance
V_max=80;
A_max=30;
L=6;%number of lanes
Mmax = L-1;
mmin = -L+1;
p_max = 1;

%------------desired states-----------
Zd = sdpvar(1,1);%carril deseado
Vd = sdpvar(1,1);%velocidad deseada
DS = sdpvar(1,1);%velocidad deseada

% -------------local vehicle---------------
v = sdpvar(ones(1,N+1),ones(1,N+1)); %velocidad del vehiculo actual
a = sdpvar(ones(1,N),ones(1,N)); %aceleracion actual del vehiculo
z = intvar(ones(1,N+1),ones(1,N+1)); %carril actual
ll = binvar(ones(1,N),ones(1,N)); %paso izquierda
lr = binvar(ones(1,N),ones(1,N)); %paso derecha
% -------------- neighboor ---------------
lr2 = binvar(1,1); %paso derecha
v_2 = sdpvar(1,1);  v_3 = sdpvar(1,1);  %velocidad del otro vehculo
z_2 = sdpvar(1,1);  z_3 = sdpvar(1,1);  %carril del vehiculo j
% ------ distance between two vehicles ------
dis12 = sdpvar(ones(1,N+1),ones(1,N+1));  %distancia entre vehiculo 1 y 2

%% neihboorhod variables 

a12 = binvar(ones(1,N),ones(1,N));     
b12 = binvar(ones(1,N),ones(1,N));   
ab12 = binvar(ones(1,N),ones(1,N));   
n12 = binvar(ones(1,N),ones(1,N));   
th12 = binvar(ones(1,N),ones(1,N));   
f12 = sdpvar(ones(1,N),ones(1,N));   
g12 = sdpvar(ones(1,N),ones(1,N));   
h12 = sdpvar(ones(1,N),ones(1,N)); 

k12 = binvar(ones(1,N),ones(1,N));     
del12 = binvar(ones(1,N),ones(1,N)); 
r12 = binvar(ones(1,N),ones(1,N));
d12 = binvar(ones(1,N),ones(1,N));
u12 = binvar(ones(1,N),ones(1,N));
v12 = binvar(ones(1,N),ones(1,N));
x12 = binvar(ones(1,N),ones(1,N));
rd12 = binvar(ones(1,N),ones(1,N));
ps12 = binvar(ones(1,N),ones(1,N));
p12 = intvar(ones(1,N),ones(1,N));
s12 = intvar(ones(1,N),ones(1,N));

% 
% l_gamma1 = binvar(ones(1,N),ones(1,N));     %l_gamma2 = binvar(ones(1,N),ones(1,N));    
% l_delta1 = binvar(ones(1,N),ones(1,N));     %l_delta2 = binvar(ones(1,N),ones(1,N));    
% l_zeta1 = binvar(ones(1,N),ones(1,N));      %l_zeta2 = binvar(ones(1,N),ones(1,N));    
% l_eta1 = binvar(ones(1,N),ones(1,N));       %l_eta2 = binvar(ones(1,N),ones(1,N));    
% l_theta1 = binvar(ones(1,N),ones(1,N));     %l_theta2 = binvar(4*ones(1,N),ones(1,N));    
% l_chi1 = binvar(ones(1,N),ones(1,N));       %l_chi2 = binvar(ones(1,N),ones(1,N));    
% l_psi1 = binvar(ones(1,N),ones(1,N));       %l_psi2 = binvar(ones(1,N),ones(1,N));    
% 
% fij1 = sdpvar(ones(1,N),ones(1,N));       fij2 = sdpvar(ones(1,N),ones(1,N));    
% gij1 = sdpvar(ones(1,N),ones(1,N));       gij2 = sdpvar(ones(1,N),ones(1,N));
% hij1 = sdpvar(ones(1,N),ones(1,N));       hij2 = sdpvar(ones(1,N),ones(1,N));
% 
% kij1 = sdpvar(ones(1,N),ones(1,N));       kij2 = sdpvar(ones(1,N),ones(1,N));    
% mij1 = sdpvar(ones(1,N),ones(1,N));       mij2 = sdpvar(ones(1,N),ones(1,N));
% pij1 = sdpvar(ones(1,N),ones(1,N));       pij2 = sdpvar(ones(1,N),ones(1,N));
% qij1 = sdpvar(ones(1,N),ones(1,N));       qij2 = sdpvar(ones(1,N),ones(1,N));    
% roij1= sdpvar(ones(1,N),ones(1,N));       roij2 = sdpvar(ones(1,N),ones(1,N));    
% wij1 = sdpvar(ones(1,N),ones(1,N));       wij2 = sdpvar(ones(1,N),ones(1,N)); 
% rij1 = sdpvar(ones(1,N),ones(1,N));       rij2 = sdpvar(ones(1,N),ones(1,N)); 
% sij1 = sdpvar(ones(1,N),ones(1,N));       sij2 = sdpvar(ones(1,N),ones(1,N)); 

 
% Aa1 = binvar( 1,N );                Aa2 = binvar( 1,N );                
% Bb1 = binvar( 1,N );                Bb2 = binvar( 1,N );                
% Gg1 = binvar( 1,N );                Gg2 = binvar( 1,N );                
% Ss1 = binvar( 1,N );                Ss2 = binvar( 1,N );                
% Nn1 = binvar( 1,N );                Nn2 = binvar( 1,N );                
  
% A1 = binvar(3*ones(1,N),ones(1,N));  A2 = binvar(3*ones(1,N),ones(1,N));
% B12 = binvar(2*ones(1,N),ones(1,N));  %B2 = binvar(2*ones(1,N),ones(1,N));  
% G1 = binvar(3*ones(1,N),ones(1,N));  %G2 = binvar(3*ones(1,N),ones(1,N));  
% D1 = binvar(5*ones(1,N),ones(1,N));  %D2 = binvar(5*ones(1,N),ones(1,N));  
% % S1 = binvar(5*ones(1,N),ones(1,N));  %S2 = binvar(5*ones(1,N),ones(1,N));  
% N1 = binvar(3*ones(1,N),ones(1,N));  %N2 = binvar(3*ones(1,N),ones(1,N));  
% Fi1 = binvar(2*ones(1,N),ones(1,N));  %Fi2 = binvar(3*ones(1,N),ones(1,N));  
% Gi1 = binvar(2*ones(1,N),ones(1,N));  %Gi2 = binvar(3*ones(1,N),ones(1,N));  
% Hi1 = binvar(2*ones(1,N),ones(1,N));  %Hi2 = binvar(3*ones(1,N),ones(1,N));  



% p_a = sdpvar(1);
% p_z = intvar(1);


%% making the optimizer with 1 node
constraints = [];
% constraints = [constraints,  diff([p_z z{1}]) == 0];
objective   = 0;

for k = 1:N
 objective = objective+( v{k+1}-Vd )'*Q*( v{k+1}-Vd ) + (z{k+1} - Zd)'*R*(z{k+1} - Zd); % calculate obj

  % Feasible region
    constraints = [constraints,z{k}-lr{k} <= z{k+1} ,
                                             z{k+1} <= z{k}+ll{k},
% constraints = [constraints,1 <=    z{k+1}     <= L,
                               1 <=    z_2      <= L,       %tome valores posibles
                                0<=    v{k+1}   <= V_max,   %no exceda las velocidades 
%                          z{k}-[p_max]<=    z{k+1}   <=z{k}+[p_max], %paso de un carril
                           -A_max<=    a{k}     <= A_max];
    
    constraints = [constraints, [1 <= z{k}    <=   L   ]];
%     constraints = [constraints, ismember(z{k},[1:1:L] ) ];
%     constraints = [constraints, z{k+1} == z{k} + ll{k} - lr{k}];
    
%     constraints = [constraints, -1 <= [z{k+1} - z{k}] <= 1];
    constraints = [constraints,  ll{k} + lr{k} <= 1];
    constraints = [constraints, v{k+1} == v{k}+T*a{k}];             %velocidad futura

   
% ---------------------------------------vehiculo 2-------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

%     constraints = [constraints, -100000  <=  dis12{k+1} <= 100000];
    constraints = [constraints,   mmin  <= z_2-z{k+1}  <= Mmax];
    constraints = [constraints, dis12{k+1} == dis12{k} + T*(v_2-v{k})];
    



%................................... (12)...............................
constraints = log_min(constraints, n12{k}, z_2-z{k} , 0);
constraints = log_may(constraints, th12{k}, z_2-z{k} , 0);
constraints = log_and(constraints, a12{k}, n12{k} , th12{k} );

%................................... alpha...............................
% constraints = log_eq(constraints, A1{k}, a12{k}, z_2-z{k} , 0);


%................................... (13)...............................
constraints = log_may(constraints, b12{k}, dis12{k} , 0);
%................................... (18)...............................
constraints = log_and(constraints, ab12{k}, a12{k} , b12{k});
%................................... (21)...............................
constraints = log_imp(constraints, f12{k}, dis12{k} , ab12{k});
%................................... (22)...............................
constraints = log_imp(constraints, g12{k}, Ds , a12{k});
%................................... (23)...............................
constraints = log_imp(constraints, h12{k}, dis12{k} , a12{k});

%................................... (24)...............................
constraints = [constraints, -2*f12{k} + g12{k} + h12{k} <= 0];



% %................................... (15)...............................
% constraints = log_min(constraints, k12{k}, z_2-z{k} , 1);
% constraints = log_may(constraints, del12{k}, z_2-z{k} , 1);
% constraints = log_and(constraints, r12{k}, k12{k} , del12{k} );
% %................................... (16)...............................
% constraints = log_and(constraints, d12{k}, ll{k} , lr2);
% % constraints = [constraints, -d12{k} + ll{k} <= 0];
% % constraints = [constraints, d12{k} - ll{k} <= 0];
% %................................... (17)...............................
% constraints = log_min(constraints, u12{k}, dis12{k} , Dl);
% constraints = log_may(constraints, v12{k}, dis12{k} , -Dl);
% constraints = log_or(constraints, x12{k}, u12{k} , v12{k} );
% %................................... (30)...............................
% constraints = log_and(constraints, rd12{k}, r12{k} , d12{k});
% %................................... (31)...............................
% constraints = log_and(constraints, ps12{k}, x12{k} , rd12{k});
% %................................... (32)...............................
% constraints = [constraints, -s12{k} + p12{k} <= 0];
% constraints = [constraints, s12{k} - p12{k} <= 0];
% %................................... (33)...............................
% constraints = log_imp(constraints, p12{k}, z{k} , ps12{k});
% %................................... (34)...............................
% constraints = log_imp(constraints, s12{k}, z{k+1} , ps12{k});

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables

end
% objective = objective+( v{N+1}-Vd )'*Q*( v{N+1}-Vd ) + (z{N+1} - Zd)'*R*(z{N+1} - Zd); % calculate obj
constraints = [constraints, [dis12{1} <= 100000]]; 
% objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
%% solver definition 2 node  

parameters_in = { Vd , Zd , v{1} , z{1} , ...
                            v_2 , z_2 , dis12{1}, lr2 } ;%, Aa1 , Bb1 , Ss1 , Nn1};%, Gg1
                         
                            
solutions_out = {[a{:}], [z{:}], [v{:}], [r12{:}] ,[d12{:}] ...
                 ,[x12{:}], [rd12{:}], [ps12{:}], [p12{:}] ...
                 ,[v12{:}], [u12{:}] ...
                 ,[ll{:}], [lr{:}]};%,  [s1{:}], [n1{:}] [g1{:}],
%                                          [a4], [dis15{:}], [b4], [g4], [z4], [n4]};
% controller = optimizer(constraints, objective,sdpsettings('verbose',1),[x{1};r],u{1});
controller1 = optimizer(constraints, objective , sdpsettings('solver','gurobi'),parameters_in,solutions_out);


%% Building variables

%define las condiciones iniciales que deben tener las variables
%logicas


 %.....................vehiculo 1..........................
 
% ..........historial de las predicciones 
 hist_vp1=[];  
 hist_zp1=[];   

 hist_vp2=[];  
 hist_zp2=[];  
 
%------condiciones iniciales----------
vel= [20; 20];% velociodad inicial
Vdes=[30; 80]; %velocidad deseada

zel= [2; 1]; %carril inicial
Zdes=[1; 1]; %carril deseado


acel=[0 0]';
%---distancia inicial de cada agente
d1i = [-50]';

% hold on
vhist = vel;
zhist = zel;
ahist = acel;
dhist = d1i;
mpciter = 0;

hist_r1 = [];
hist_d1 = [];
hist_x1 = [];
hist_rd1 = [];
hist_ps1 = [];
hist_p1 = [];
hist_s1 = [];
hist_del1 = [];


hist_r2 = [];
hist_d2 = [];
hist_x2 = [];
hist_rd2 = [];
hist_ps2 = [];
hist_p2 = [];
hist_v2 = [];
hist_ul2 = [];


hist_ll1 = [];
hist_lr1 = [];
hist_ll2 = [];
hist_lr2 = [];
 
 i=0;
 zel2 = zel;   %same dimentions
 time=20;
 tic
 sim_tim = 20;
 LR2 = [1];
 LR1 = [1];
for i = 1 : 9

%.........................      solver vehiculo 1       ............................


        inputs = {Vdes(1) , Zdes(1) , vel(1) , zel(1) ,  ...
                   vel(2) , zel(2)  , d1i(1), LR2};%, alogic1_1 , blogic1_1  , S1logic_1 , N1logic_1}; 
    [solutions1,diagnostics] = controller1{inputs};    
     
    A =  solutions1{1};         acel(1) = A(:,1);
    Z =  solutions1{2};         zel(1)=Z(:,2);                  hist_zp1=[hist_zp1; Z];
    V =  solutions1{3};         hist_vp1 = [hist_vp1; V];
    Aa = solutions1{4};         hist_r1 = [hist_r1; Aa ];
    B =  solutions1{5};         hist_d1 = [hist_d1; B ];
    CC = solutions1{6};         hist_x1 = [hist_x1; CC ];
    DD = solutions1{7};         hist_rd1 = [hist_rd1; DD ];
    EE = solutions1{8};         hist_ps1 = [hist_ps1; EE ];
    FF = solutions1{9};         hist_p1 = [hist_p1; FF ];
    GG = solutions1{10};        hist_s1 = [hist_s1; GG ];
    HH = solutions1{11};        hist_del1 = [hist_del1; HH ];
    II = solutions1{12};        hist_ll1 = [hist_ll1; II ];       
    JJ = solutions1{13};        hist_lr1 = [hist_lr1; JJ ];     LR1 = JJ(:,1);
%     KK = solutions1{14};        LR1 = KK(:,1);                  hist_lr1 = [hist_lr1; KK ];
%     fij1_hist = [fij1_hist; solutions1{15} ];
%     gij1_hist = [gij1_hist; solutions1{16} ];
%     hij1_hist = [hij1_hist; solutions1{17} ];
    
    
%   Gg1 = solutions1{6};    g1hist_1=[g1hist_1 Gg1];        G1logic_1=Gg1;

    if diagnostics == 1
        error('you are close, keep trying 1');
    end   
    


    
%.........................      solver vehiculo 2       ............................

        inputs = {Vdes(2) , Zdes(2) , vel(2) , zel(2) ,...
                   vel(1) , zel(1)  , -d1i(1) , LR1};%, alogic1_2 , blogic1_2  , S1logic_2 , N1logic_2}; %G1logic_1
    [solutions2,diagnostics] = controller1{inputs};    
    
    A =  solutions2{1};         acel(2) = A(:,1);
    Z =  solutions2{2};         zel(2)=Z(:,2);                  hist_zp2=[hist_zp2; Z];
    V =  solutions2{3};         hist_vp2 = [hist_vp2; V];
    Aa = solutions2{4};         hist_r2 = [hist_r2; Aa ];
    B =  solutions2{5};         hist_d2 = [hist_d2; B ];
    CC = solutions2{6};         hist_x2 = [hist_x2; CC ];
    DD = solutions2{7};         hist_rd2 = [hist_rd2; DD ];
    EE = solutions2{8};         hist_ps2 = [hist_ps2; EE ];
    FF = solutions2{9};         hist_p2 = [hist_p2; FF ];
    GG = solutions2{10};        hist_v2 = [hist_v2; GG ];
    HH = solutions2{11};        hist_ul2 = [hist_ul2; HH ];
    II = solutions2{12};        hist_ll2 = [hist_ll2; II ];       
    JJ = solutions2{13};        hist_lr2 = [hist_lr2; JJ ];     LR2 = JJ(:,1);
    
    
    
    if diagnostics == 1
        error('you are close, keep trying 2');
    end
    


    
    %----------------------------------------------------------------------
%     zel= zel2;
    d1i = d1i + T*(vel(2:nv ) - ones((nv-1),1)*vel(1));
%     d2i = [-d1i(1); -d1i(1)+d1i(2)];
    
    vel = vel+T*acel;
    vhist = [vhist vel];
    zhist = [zhist zel];
    ahist = [ahist acel];
    dhist = [dhist; d1i];
    
%     pause(0.05)

mpciter;
mpciter = mpciter + 1;
end
toc

disp("it's done")

%% plot 

vphist=cat(3, hist_vp1 , hist_vp1);
zphist=cat(3, hist_zp1 , hist_zp2);

 Draw_object(vhist,zhist,vphist,zphist,dhist,T, 0)
% save('myFile5.mat','vhist','zhist','vphist','zphist','dhist','T')