%UNIVERSIDAD NACIONAL DE COLOMBIA
% Multi Vehicle automated drivring
%Autor: Nestor Ospina
clear 
close all
clc

% %---------laptop asus
% addpath(genpath('C:\gurobi901\win64\matlab'))%GUROBI
% addpath(genpath('C:\Users\nesto\OneDrive\Documentos\YALMIP-master'))%yalmip

% ------------------------------------------------------------------------------------------------------------------------

yalmip('clear')
%% PROGRAM 
% Model data
nx = 1; % Number of agents
nu = 1; % Number of inputs
nv=1; %numero de vehiculos sin el agente no cooperativo
m=1; %masa del vehiculo
lf=1; %distance from center to front
lr=1; %distance from center to rear
iz = 1; %Inertia coefficient


% MPC data
Q = 1*eye(3);
Q(3,3) = 1;
R = 1*eye(2);
N = 5;%horizon
t = 0.5; %[s]
Ds=15;%Safety distance
Dl=15; %lateral distance safe
V_max=80;
T_max=1;
L=6; %number of lanes
Mmax=L-1;
mmin=-L+1;
p_max=0.3; %paso maximo de un carril al otro


%------------estados deseados-----------
 Yd = sdpvar(1,1);%carril deseado
 vxd = sdpvar(1,1);%velocidad deseada
 ang_d = sdpvar(1,1);% desired angle 


% -------------estados vehiculo i---------------
v = sdpvar(2*ones(1,N+1),ones(1,N+1)); %velocidad local del vehiculo [vy vx]
ang = sdpvar(2*ones(1,N+1),ones(1,N+1)); %Position angular [fi dfi]
p = sdpvar(2*ones(1,N+1),ones(1,N+1)); %Position GLOBAL del vehiculo [X Y]

% a = sdpvar(2*ones(1,N+1),ones(1,N+1)); %aceleracion del vehiculo actual

tf = sdpvar(2*ones(1,N+1),ones(1,N+1)); %torque  [Txf Tyf]'
tr = sdpvar(2*ones(1,N+1),ones(1,N+1)); %torque  [Txr Tyr]'

% tr = sdpvar(2*ones(1,N+1),ones(1,N+1)); %front torque
% z = sdpvar(ones(1,N+1),ones(1,N+1)); %carril actual


%  
% p_tf = sdpvar(1);
% p_tr = sdpvar(1);


%% making the optimizer with 1 node ------------------------------------------------------------
constraints = [];
% constraints = [constraints,  diff([p_tf tf{1}]) == 0];
% constraints = [constraints,  diff([p_tr tr{1}]) == 0];
objective   = 0;

for k = 1:N
 objective = objective + ([v{k}(1) p{k}(2) ang{k}(1)]-[vxd Yd ang_d])*Q*([v{k}(1) p{k}(2) ang{k}(1)]-[vxd Yd ang_d])' + tf{k}'*R*tf{k} + tr{k}'*R*tr{k}; % calculate obj

  % Feasible region
    constraints = [constraints,1 <=    p{k+1}(2)     <= L, % it keep on the way
                                0<=    v{k+1}        <= V_max,
                              -pi<=    ang{k}      <= pi,
                              -pi<=    ang{k+1}    <= pi,
                p{k}(2) - [p_max]<=    p{k+1}(2)     <= p{k}(2) + [p_max], %paso de un carril
                           -T_max<=    tf{k}         <= T_max,
                           -T_max<=    tr{k}         <= T_max];
    
    constraints = [constraints,[1 <=  p{k}(2)  <= L ]]; % it keep on the way
     
     %Vehicle Dynamic
     constraints = [constraints, v{k+1}(2) == v{k}(2) - t*ang{k}(2)*v{k}(1) - t*v{k}(1)*ang{k}(2) + (2/m)*tf{k}(2)*t + (2/m)*tr{k}(2)*t] ;
     constraints = [constraints, v{k+1}(1) == v{k}(1) + t*ang{k}(2)*v{k}(2) + t*v{k}(2)*ang{k}(2) + (2/m)*tf{k}(1)*t + (2/m)*tf{k}(1)*t] ;
     constraints = [constraints, ang{k+1}(1) == ang{k}(1) + t*ang{k}(2)];
     constraints = [constraints, ang{k+1}(2) == ang{k}(2) + (2*lf/iz)*t*tf{k}(2) - (2*lr/iz)*t*tr{k}(2)];
     constraints = [constraints,  p{k+1}(2) == t*cos(ang{k}(1))*v{k}(2) + t*sin(ang{k}(1))*v{k}(1) + angle(1)*v{k}(1)*cos(ang{k}(1))*t - ang{k}(1)*v{k}(2)*sin(ang{k}(1))*t +  p{k}(2)];
     constraints = [constraints,  p{k+1}(1) == -t*sin(ang{k}(1))*v{k}(2) - t*cos(ang{k}(1))*v{k}(1) - angle(1)*v{k}(1)*sin(ang{k}(1))*t - ang{k}(1)*v{k}(2)*cos(ang{k}(1))*t +  p{k}(1)];
     % It is EXTREMELY important to add as many
     % constraints as possible to the binary variables
end


%% solver definition 1 node  ------------------------------------------------------------

parameters_in = { Yd ,vxd , ang_d ,...    
                        v{1} , ang{1} , p{1}};%, Gg1

solutions_out = {[tf{:}], [tr{:}], [v{:}], [p{:}]};

controller1 = optimizer(constraints, objective , sdpsettings('solver','BMIBNB'),parameters_in,solutions_out);



%% Building variables

%define las condiciones iniciales que deben tener las variables
%logicas


% 
% %------------estados deseados-----------
%  Yd = sdpvar(1,1);%carril deseado
%  vxd = sdpvar(1,1);%velocidad deseada
%  ang_d = sdpvar(1,1);% desired angle 
% 
% 
% % -------------estados vehiculo i---------------
% v = sdpvar(2*ones(1,N+1),ones(1,N+1)); %velocidad local del vehiculo [vy vx]
% ang = sdpvar(2*ones(1,N+1),ones(1,N+1)); %Position angular [fi dfi]
% p = sdpvar(2*ones(1,N+1),ones(1,N+1)); %Position GLOBAL del vehiculo [X Y]

%------condiciones iniciales----------
vel= [0; 0];% velociodad inicial [vy vx]
Vdes=[20]; %velocidad deseada

angle = [0; 0]; % angulo inicial [fi dfi]
ang_des = [0]; %angulo deseado

pos = [0; 0]; %posicion inicial
ydes = [3]; %carril deseado

i=0;

 time=20;
 tic
 
for ii = 1 : 30
% while ( vel-Vdes )'*Q*( vel-Vdes ) + (zel - Zdes)'*R*(zel - Zdes) - p_optima > epsilon
 i=i+1;

%  parameters_in = { Yd ,vxd , ang_d , tf, tr,...    
%                         v{1} , ang{1} , p{1}};%, Gg1
% 
% solutions_out = {[tf{:}], [tr{:}], [v{:}], [p{:}]};

 %.........................      solver vehiculo 1       ............................
        inputs = {ydes , Vdes , ang_des,...
                   vel , angle  , pos}; 
    [solutions1,diagnostics] = controller1{inputs};    

    TF =  solutions1{1};         tfront = TF(:,1);
    TR =  solutions1{2};         trear=TR(:,1);                 % zp1hist=[zp1hist; Z];
    V =  solutions1{3};          vel = V(:,1);
    P = solutions1{4};           pos = P(:,1);

    if diagnostics == 1
        error('you are close, keep trying 1');
    end   

    vhist = [vhist vel];
    zhist = [zhist zel];
    ahist = [ahist acel];
    dhist = [dhist d1i];

 vel(2) == vel(2) - t*angle{k}(2) - t*vel(1) + (2/m)*tfront(2)*t + (2/m)*trear(2)*t ;
 vel(1) == vel(1) + t*angle{k}(2) - t*vel(2) + (2/m)*tfront(1)*t + (2/m)*trear(1)*t ;
 angle(1) == angle(1) + t*angle(2);  
 angle(2) == angle(2) + (2*lf/iz)*t*tfront(2) + (2*lr/iz)*t*trear(2);  
 pos(2) == t*cos(angle(1))*vel(2) + t*sin(angle(1))*v{k}(1) + angle(1)*vel(1)*cos(angle(1))*t - angle(1)*vel(2)*sin(angle(1))*t + pos(2);
 pos(1) == - t*sin(angle(1))*vel(1) - t*cos(angle(1))*v{k}(1) - angle(1)*vel(1)*sin(angle(1))*t - angle(1)*vel(2)*cos(angle(1))*t + pos(1);             

    
%     pause(0.05)   


end
toc

% vphist=cat(3, vp1hist , vp2hist, vp3hist, vp4hist, vp5hist);
% zphist=cat(3, zp1hist , zp2hist, zp3hist, zp4hist, zp5hist);

% Draw_basico(vhist,zhist,...                     
%                             vp1hist,zp1hist,...
%                             vp2hist,zp2hist,...
%                             vp3hist,zp3hist,...
%                                                  dhist,T,N)


Draw_object(vhist,zhist,vphist,zphist,dhist,t)
save('myFile5.mat','vhist','zhist','vphist','zphist','dhist','T')
