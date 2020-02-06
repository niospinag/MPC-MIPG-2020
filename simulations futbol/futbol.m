
% %UNIVERSIDAD NACIONAL DE COLOMBIA
% futboll game
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


addpath(genpath('/opt/gurobi900/linux64'))%gurobi
% addpath(genpath('/opt/ibm/ILOG/CPLEX_Studio_Community129/cplex/matlab/x86-64_linux'))%cplex
 addpath(genpath('/home/tavocardona/Documents/YALMIP-master'))%yalmip
yalmip('clear')


%% PROGRAM 
% Model data
nv=1; %numero de vehiculos sin el agente no cooperativo
% MPC data
Q = eye(2);
Q1 = eye(1);
R = eye(2);% numero de 
N = 10;%horizon
T = 0.5; %[s]
V_max=1;
e=1;
Rr = 0.8; %radio del robot
r_arc = 3;
%i=1;


% ------------- estados del jugador ---------------
p = sdpvar(2*ones(1,N+1),ones(1,N+1)); %picion en x y Y
vel = sdpvar(2*ones(1,N+1),ones(1,N+1)); % velocidad en X y Y


% ------------- estados de los jugadores del mismo equipo -----
p_2= sdpvar(2,1); % picion del otro jugador
p_3= sdpvar(2,1); % picion del otro jugador
p_4= sdpvar(2,1); % picion del otro jugador
p_5= sdpvar(2,1); % picion del otro jugador

% -------- estados de los jugadores del equipo contrario --------
p_op1 = sdpvar(2,1); 
p_op2 = sdpvar(2,1); 
p_op3 = sdpvar(2,1); 


%----------- mixed-integer variables -------------

str1 = binvar(1);       f = binvar(1);



p_vel = sdpvar(2,1);
p_pos = sdpvar(2,1);

constraints = -0.2 <= diff([p_vel vel{:}]) <= 0.2;
constraints = [constraints, -0.1 <= diff([p_pos p{:}]) <= 0.1];
objective   = 0;

%-----creacion de funcion objetivo y restricciones--------------------
for k = 1:N
 objective = objective +  (1- str1*f)*((atan(p{k}(2)/p{k}(1))-atan(p_op1(2)/p_op1(1)))'*Q1*((atan(p{k}(2)/p{k}(1))-atan(p_op1(2)/p_op1(1))))) + ...
                          (1- str1*(1-f))*((p{k})'*Q*(p{k})) + ...
                          (vel{k})'*R*(vel{k}); % calculate obj
  
% 
    constraints = [constraints,[-10; 0] <=    p{k}    <= [10; 50], % no se salga de la cancha
                      -[V_max; V_max] <=    vel{k}      <= [V_max; V_max]];% no exceda la velocidad
  
% restricciones de distancia con los otros agentes                      
    constraints = [constraints,[            2*Rr        <= sqrt( (p{k+1}(1)-p_2(1))^2 +p{k+1}(2)-p_2(2)^2 ),... 
                                            2*Rr        <= sqrt( (p{k+1}(1)-p_3(1))^2 +p{k+1}(2)-p_3(2)^2 ),...
                                            2*Rr        <= sqrt( (p{k+1}(1)-p_4(1))^2 +p{k+1}(2)-p_3(2)^2 ),...
                                            2*Rr        <= sqrt( (p{k+1}(1)-p_5(1))^2 +p{k+1}(2)-p_3(2)^2 )] ];
                               
  
                                        
                                        
%.........................alpha...............................

% constraints = [constraints, [D1(1,1,i)+D1(2,1,i)+D1(3,1,i)==1],... 
%               implies( D1(1,1,i), [ a1==0, z_2-z{k} <=-0.1 ]);
%               implies( D1(2,1,i), [ a1==1, -0.1<=z_2-z{k} <=0.1 ]);
%               implies( D1(3,1,i), [ a1==0, 0.1 <= z_2-z{k} ]) ];
%           
    % definicion de la dinamica interna de cada agente    
    constraints = [ constraints, p{k+1} == p{k}+T*vel{k} ]; % posicion futura
    
    %restriccion de distancia del arco
     constraints = [constraints,  Rr +r_arc<= sqrt( (p{k+1}(1)-p_2(1))^2 + (p{k+1}(2)-p_2(2))^2 )]; 
    
  
   
% ------------------------------------vehiculo 2-------------------------------    
%------------------si dz=0  -------------------->>>    dij>= Ds----------------






    % It is EXTREMELY important to add as many
    % constraints as psible to the binary variables
    
end


% objective = objective+(vel{N+1}-Vd)'*Q*(vel{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
%% solver definition   

parameters_in = {p_pos{1}, str1, f, p_2, p_3, p_4, p_5};

                       

                            
solutions_out = {[vel{:}] };
                               

controller1 = optimizer(constraints, objective,sdpsettings('solver','gurobi'),parameters_in,solutions_out);
%------condiciones iniciales----------
pos=[5; 5];
pos_2=[2; 8];
pos_3=[3; 10];
pos_4=[4; 12];
pos_5=[5; 7];

str1 = 1;
f = 1;


%                 Vdes=[10; 35; 35; 15]; %velocidad deseada
%                 Zdes=[5; 2; 3; 4];




%define las condiciones iniciales que deben tener las variables logicas

%.....................vehiculo 1.....................................................vehiculo 2
 

for i = 1:30



%.........................solver vehiculo 1............................

% parameters_in = {p{1}, str1, f, p_2, p_3, p_4, p_5};


inputs = {pos(:),str1,f,pos_2,pos_3,pos_4,pos_5};
[solutions1,diagnostics] = controller1{inputs};

% solutions_out = {[vel{:}], [p{:}] };

    VEL = solutions1{1}; %past_a(1) = A(:,1);
    POS = solutions1{2};   zel(1)=Z(:,1);             zp1hist=[zp1hist; Z];
    V = solutions1{3};   vp1hist=[vp1hist; V];
    A1 = solutions1{4};                             alogic1_1=A1(:,1);
        
    
    if diagnostics == 1
        error('you are close, keep trying 1');
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

