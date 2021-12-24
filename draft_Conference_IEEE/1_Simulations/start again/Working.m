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
T = 0.1; %[s]
Ds=15;%Safety distance [m]
Dl=25; %lateral distance
V_max=80;
A_max=30;
L=6;%number of lanes
Mmax = L-1;
mmin = -L+1;
p_max = 1;


%------------estados deseados-----------
 Zd = sdpvar(1,1);%carril deseado
 Vd = sdpvar(1,1);%velocidad deseada

% -------------vehiculo i---------------
v = sdpvar(ones(1,N+1),ones(1,N+1)); %velocidad del vehiculo actual
a = sdpvar(ones(1,N+1),ones(1,N+1)); %aceleracion actual del vehiculo
z = intvar(ones(1,N+1),ones(1,N+1)); %carril actual
ll = binvar(ones(1,N),ones(1,N)); %paso izquierda
lr = binvar(ones(1,N),ones(1,N)); %paso derecha

lr2 = binvar(1,1); %paso derecha


v_2 = sdpvar(1,1);  v_3 = sdpvar(1,1);  %velocidad del otro vehculo
z_2 = sdpvar(1,1);  z_3 = sdpvar(1,1);  %carril del vehiculo j

dis12 = sdpvar(ones(1,N+1),ones(1,N+1));  %distancia entre vehiculo 1 y 2

l_alpha1 = binvar(ones(1,N),ones(1,N));     l_alpha2 = binvar(ones(1,N),ones(1,N));
l_beta1 = binvar(ones(1,N),ones(1,N));      l_beta2 = binvar(ones(1,N),ones(1,N));    
l_gamma1 = binvar(ones(1,N),ones(1,N));     l_gamma2 = binvar(ones(1,N),ones(1,N));    
l_delta1 = binvar(ones(1,N),ones(1,N));     l_delta2 = binvar(ones(1,N),ones(1,N));    
l_zeta1 = binvar(ones(1,N),ones(1,N));      l_zeta2 = binvar(ones(1,N),ones(1,N));    
l_eta1 = binvar(ones(1,N),ones(1,N));       l_eta2 = binvar(ones(1,N),ones(1,N));    
l_theta1 = binvar(ones(1,N),ones(1,N));     l_theta2 = binvar(4*ones(1,N),ones(1,N));    
l_chi1 = binvar(ones(1,N),ones(1,N));       l_chi2 = binvar(ones(1,N),ones(1,N));    
l_psi1 = binvar(ones(1,N),ones(1,N));       l_psi2 = binvar(ones(1,N),ones(1,N));    

fij1 = sdpvar(ones(1,N),ones(1,N));       fij2 = sdpvar(ones(1,N),ones(1,N));    
gij1 = sdpvar(ones(1,N),ones(1,N));       gij2 = sdpvar(ones(1,N),ones(1,N));
hij1 = sdpvar(ones(1,N),ones(1,N));       hij2 = sdpvar(ones(1,N),ones(1,N));

kij1 = sdpvar(ones(1,N),ones(1,N));       kij2 = sdpvar(ones(1,N),ones(1,N));    
mij1 = sdpvar(ones(1,N),ones(1,N));       mij2 = sdpvar(ones(1,N),ones(1,N));
pij1 = sdpvar(ones(1,N),ones(1,N));       pij2 = sdpvar(ones(1,N),ones(1,N));
qij1 = sdpvar(ones(1,N),ones(1,N));       qij2 = sdpvar(ones(1,N),ones(1,N));    
roij1= sdpvar(ones(1,N),ones(1,N));       roij2 = sdpvar(ones(1,N),ones(1,N));    
wij1 = sdpvar(ones(1,N),ones(1,N));       wij2 = sdpvar(ones(1,N),ones(1,N)); 
rij1 = sdpvar(ones(1,N),ones(1,N));       rij2 = sdpvar(ones(1,N),ones(1,N)); 
sij1 = sdpvar(ones(1,N),ones(1,N));       sij2 = sdpvar(ones(1,N),ones(1,N)); 

 
% Aa1 = binvar( 1,N );                Aa2 = binvar( 1,N );                
% Bb1 = binvar( 1,N );                Bb2 = binvar( 1,N );                
% Gg1 = binvar( 1,N );                Gg2 = binvar( 1,N );                
% Ss1 = binvar( 1,N );                Ss2 = binvar( 1,N );                
% Nn1 = binvar( 1,N );                Nn2 = binvar( 1,N );                
  
A1 = binvar(3*ones(1,N),ones(1,N));  A2 = binvar(3*ones(1,N),ones(1,N));
B1 = binvar(2*ones(1,N),ones(1,N));  B2 = binvar(2*ones(1,N),ones(1,N));  
G1 = binvar(3*ones(1,N),ones(1,N));  G2 = binvar(3*ones(1,N),ones(1,N));  
D1 = binvar(5*ones(1,N),ones(1,N));  D2 = binvar(5*ones(1,N),ones(1,N));  
% S1 = binvar(5*ones(1,N),ones(1,N));  S2 = binvar(5*ones(1,N),ones(1,N));  
N1 = binvar(3*ones(1,N),ones(1,N));  N2 = binvar(3*ones(1,N),ones(1,N));  
Fi1 = binvar(2*ones(1,N),ones(1,N));  Fi2 = binvar(3*ones(1,N),ones(1,N));  
Gi1 = binvar(2*ones(1,N),ones(1,N));  Gi2 = binvar(3*ones(1,N),ones(1,N));  
Hi1 = binvar(2*ones(1,N),ones(1,N));  Hi2 = binvar(3*ones(1,N),ones(1,N));  



% p_a = sdpvar(1);
% p_z = intvar(1);


%% making the optimizer with 1 node
constraints = [];
% constraints = [constraints,  diff([p_z z{1}]) == 0];
objective   = 0;

for k = 1:N
 objective = objective+( v{k+1}-Vd )'*Q*( v{k+1}-Vd ) + (z{k+1} - Zd)'*R*(z{k+1} - Zd); % calculate obj

  % Feasible region
    constraints = [constraints,1 <=    z{k+1}     <= L,
                               1 <=    z_2      <= L,       %tome valores posibles
                                0<=    v{k+1}   <= V_max,   %no exceda las velocidades 
%                          z{k}-[p_max]<=    z{k+1}   <=z{k}+[p_max], %paso de un carril
                           -A_max<=    a{k}     <= A_max];
    
    constraints = [constraints, [1 <= z{1}    <=   L   ]];
    
    constraints = [constraints, z{k+1} == z{k} + ll{k} - lr{k}];
    
    constraints = [constraints, -1 <= [z{k+1} - z{k}] <= 1];
    constraints = [constraints,  ll{k} + lr{k} <= 1];
    constraints = [constraints, v{k+1} == v{k}+T*a{k}];             %velocidad futura

   
% ---------------------------------------vehiculo 2-------------------------------    
% ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, -100000  <=  dis12{k+1} <= 100000];
    constraints = [constraints,   mmin  <= z_2-z{k+1}  <= Mmax];
    constraints = [constraints, dis12{k+1} == dis12{k} + T*(v_2-v{k})];
    constraints = [constraints, [dis12{1} <= 100000]]; 



%................................... alpha...............................
constraints = log_eq(constraints, A1{k}, l_alpha1{k}, z_2-z{k} , 0);

%................................... Beta ....................................
constraints = log_may(constraints, B1{k}, l_beta1{k}, dis12{k},0);

%................................... Gamma ....................................
constraints = log_eq(constraints, G1{k}, l_gamma1{k}, z_2-z{k+1},0);

%................................... Delta ....................................
constraints = log_eq1(constraints, D1{k}, l_delta1{k}, z_2-z{k},1);

%................................... Zeta ....................................
% constraints = log_and(constraints, l_delta1{k}, ll{k}, lr2);

%................................... Eta ....................................
constraints = log_or(constraints, N1{k}, l_eta1{k}, dis12{k}, Dl );

%................................... Theta ....................................
constraints = log_and(constraints, l_theta1{k}, l_alpha1{k} , l_beta1{k} );

%...................................  Chi ....................................
constraints = log_and(constraints, l_chi1{k}, l_theta1{k} , l_gamma1{k} );

%................................... Psi ....................................
constraints = log_and(constraints, l_psi1{k}, l_alpha1{k} , l_gamma1{k} );

%................................... Fij ....................................
constraints = log_imp(constraints, fij1{k}, dis12{k}, l_theta1{k});

%................................... Gij ....................................
constraints = log_imp(constraints, gij1{k}, Ds, l_alpha1{k});

%................................... Hij ....................................
constraints = log_imp(constraints, hij1{k}, dis12{k}, l_alpha1{k});

%................................... constraint (10a) .......................
constraints = [constraints, -2*fij1{k} + gij1{k} + hij1{k} <= 0];

%................................... Kij ....................................
constraints = log_imp(constraints, kij1{k}, v_2-v{k}, l_chi1{k});

%................................... Mij ....................................
constraints = log_imp(constraints, mij1{k}, dis12{k}, l_chi1{k});

%................................... Pij ....................................
constraints = log_imp(constraints, pij1{k}, v_2-v{k}, l_psi1{k});

%................................... Qij ....................................
constraints = log_imp(constraints, qij1{k}, dis12{k}, l_psi1{k});

%................................... Constraint (10b) ....................................
constraints = [constraints, -2*(T*kij1{k} + mij1{k}) + qij1{k} + T*pij1{k} <= 0];

%......................................... Ro ........................................
constraints = log_and(constraints, roij1{k}, l_delta1{k} , l_chi1{k} );

%....................................... Omega ........................................
constraints = log_and(constraints, wij1{k}, v_2-v{k}, l_eta1{k} );

%................................... constraint 33 ....................................
constraints = [constraints, -sij1{k} + rij1{k} <= 0 ;
                             sij1{k} - rij1{k} <= 0 ];

%....................................... Rij ........................................
constraints = log_imp(constraints, rij1{k},  z{k},   wij1{k} );

%....................................... Sij ........................................
constraints = log_imp(constraints, sij1{k}, z{k+1},  wij1{k} );

%................................... constraint (10a) ....................................
% constraints = [constraints, l_delta1{k}*lr{k}*l_eta1{k}*() == 0];

implies
          
%           


    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
    
end
objective = objective+( v{N+1}-Vd )'*Q*( v{N+1}-Vd ) + (z{N+1} - Zd)'*R*(z{N+1} - Zd); % calculate obj

% objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
%% solver definition 2 node  

parameters_in = { Vd , Zd , v{1} , z{1} , ...
                            v_2 , z_2 , dis12{1}, lr2 } ;%, Aa1 , Bb1 , Ss1 , Nn1};%, Gg1
                         
                            
solutions_out = {[a{:}], [z{:}], [v{:}], [l_alpha1{:}] ,[l_beta1{:}] ...
                 ,[l_chi1{:}], [l_delta1{:}], [l_eta1{:}], [l_gamma1{:}] ...
                 ,[l_psi1{:}], [l_theta1{:}], [l_zeta1{:}], [ll{:}], [lr{:}] ...
                 ,[fij1{:}], [gij1{:}], [hij1{:}] };%,  [s1{:}], [n1{:}] [g1{:}],
%                                          [a4], [dis15{:}], [b4], [g4], [z4], [n4]};

controller1 = optimizer(constraints, objective , sdpsettings('solver','gurobi'),parameters_in,solutions_out);


%% Building variables

%define las condiciones iniciales que deben tener las variables
%logicas


 %.....................vehiculo 1..........................
 
% ..........historial de las predicciones 
 vp1hist=[];  
 zp1hist=[];   

 vp2hist=[];  
 zp2hist=[];  
 
%------condiciones iniciales----------
vel= [20; 20];% velociodad inicial
Vdes=[30; 80]; %velocidad deseada

zel= [5; 4]; %carril inicial
Zdes=[2; 2]; %carril deseado


acel=[0 0]';
%---distancia inicial de cada agente
d1i = [-30]';

% hold on
vhist = vel;
zhist = zel;
ahist = acel;
dhist = d1i;
mpciter = 0;
a1_hist = [];
a2_hist = [];
b1_hist = [];
b2_hist = [];
chi1_hist = [];
delta1_hist = [];
eta1_hist = [];
gamma1_hist = [];
psi1_hist = [];
theta1_hist = [];
zeta1_hist = [];
chi2_hist = [];
delta2_hist = [];
eta2_hist = [];
gamma2_hist = [];
psi2_hist = [];
theta2_hist = [];
zeta2_hist = [];
ll1_hist = [];
lr1_hist = [];
ll2_hist = [];
lr2_hist = [];

fij1_hist = [];
gij1_hist = [];
hij1_hist = [];

fij2_hist = [];
gij2_hist = [];
hij2_hist = [];
 
 i=0;
 zel2 = zel;   %same dimentions
 time=20;
 tic
 sim_tim = 20;
 LR2 = [1]
 LR1 = [1]
for i = 1 : 25
% while ( vel-Vdes )'*Q*( vel-Vdes ) + (zel - Zdes)'*R*(zel - Zdes) - p_optima > epsilon && mpciter < sim_tim / T
 i=i+1;
%.........................      solver vehiculo 1       ............................


        inputs = {Vdes(1) , Zdes(1) , vel(1) , zel(1) , ...
                   vel(2) , zel(2)  , d1i(1), LR2};%, alogic1_1 , blogic1_1  , S1logic_1 , N1logic_1}; 
    [solutions1,diagnostics] = controller1{inputs};    
     
    A =  solutions1{1};         acel(1) = A(:,1);
    Z =  solutions1{2};         zel(1)=Z(:,2);                  zp1hist=[zp1hist; Z];
    V =  solutions1{3};         vp1hist = [vp1hist; V];
    Aa = solutions1{4};         a1_hist = [a1_hist; Aa ];
    B =  solutions1{5};         b1_hist = [b1_hist; B ];
    CC = solutions1{6};         chi1_hist = [chi1_hist; CC ];
    DD = solutions1{7};         delta1_hist = [delta1_hist; DD ];
    EE = solutions1{8};         eta1_hist = [eta1_hist; EE ];
    FF = solutions1{9};         gamma1_hist = [gamma1_hist; FF ];
    GG = solutions1{10};        psi1_hist = [psi1_hist; GG ];
    HH = solutions1{11};        theta1_hist = [theta1_hist; HH ];
    II = solutions1{12};        zeta1_hist = [zeta1_hist; II ];
    JJ = solutions1{13};        ll1_hist = [ll1_hist; JJ ];
    KK = solutions1{14};        LR1 = KK(:,1);                  lr1_hist = [lr1_hist; KK ];
    fij1_hist = [fij1_hist; solutions1{15} ];
    gij1_hist = [gij1_hist; solutions1{16} ];
    hij1_hist = [hij1_hist; solutions1{17} ];
    
    
%   Gg1 = solutions1{6};    g1hist_1=[g1hist_1 Gg1];        G1logic_1=Gg1;

    if diagnostics == 1
        error('you are close, keep trying 1');
    end   
    


    
%.........................      solver vehiculo 2       ............................

        inputs = {Vdes(2) , Zdes(2) , vel(2) , zel(2) , ...
                   vel(1) , zel(1)  , -d1i(1) , LR1};%, alogic1_2 , blogic1_2  , S1logic_2 , N1logic_2}; %G1logic_1
    [solutions2,diagnostics] = controller1{inputs};    
     
    A = solutions2{1};          acel(2) = A(:,1);
    Z = solutions2{2};          zel(2)=Z(:,2);                      zp2hist=[zp2hist; Z];
    V = solutions2{3};          vp2hist =  [vp2hist; V];        
    Aa = solutions2{4};         a2_hist = [a2_hist; Aa ];
    B = solutions2{5};          b2_hist = [b2_hist; B ];
    CC = solutions2{6};         chi2_hist = [chi2_hist; CC ];
    DD = solutions2{7};         delta2_hist = [delta2_hist; DD ];
    EE = solutions2{8};         eta2_hist = [eta2_hist; EE ];
    FF = solutions2{9};         gamma2_hist = [gamma2_hist; FF ];
    GG = solutions2{10};        psi2_hist = [psi2_hist; GG ];
    HH = solutions2{11};        theta2_hist = [theta2_hist; HH ];
    II = solutions2{12};        zeta2_hist = [zeta2_hist; II ];
    JJ = solutions2{13};        ll2_hist = [ll2_hist; JJ ];
    KK = solutions2{14};        LR2 = KK(:,1);                  lr2_hist = [lr2_hist; KK ];
 
    fij2_hist = [fij2_hist; solutions2{15} ];
    gij2_hist = [gij2_hist; solutions2{16} ];
    hij2_hist = [hij2_hist; solutions2{17} ];
    
    
    
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
    dhist = [dhist d1i];
    
%     pause(0.05)

mpciter;
mpciter = mpciter + 1;
end
toc

disp("it's done")

vphist=cat(3, vp1hist , vp2hist);
zphist=cat(3, zp1hist , zp2hist);

 Draw_object(vhist,zhist,vphist,zphist,dhist,T, 0)
% save('myFile5.mat','vhist','zhist','vphist','zphist','dhist','T')