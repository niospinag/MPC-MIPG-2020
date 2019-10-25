
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
addpath(genpath('/home/tavocardona/gurobi811/linux64'))%cplex
% addpath(genpath('/opt/ibm/ILOG/CPLEX_Studio_Community129/cplex/matlab/x86-64_linux'))%cplex
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
v = sdpvar(ones(1,N+1),ones(1,N+1)); %velocidad del vehiculo actual
a = sdpvar(ones(1,N+1),ones(1,N+1)); %aceleracion actual del vehiculo
z = sdpvar(ones(1,N+1),ones(1,N+1)); %carril actual

v_2 = sdpvar(1,nv);     v_3 = sdpvar(1,nv); %velocidad del otro vehculo
v_4 = sdpvar(1,nv);     v_4 = sdpvar(1,nv); %velocidad del otro vehculo
z_2 = sdpvar(1,nv);     z_3 = sdpvar(1,nv); %carril del vehiculo j
z_4 = sdpvar(1,nv);     z_5 = sdpvar(1,nv); %carril del vehiculo j
dis12 = sdpvar(ones(1,N+1)*nv,ones(1,N+1));     dis13 = sdpvar(ones(1,N+1)*nv,ones(1,N+1)); %distancia entre vehiculo 1 y 2
dis14 = sdpvar(ones(1,N+1)*nv,ones(1,N+1));     dis15 = sdpvar(ones(1,N+1)*nv,ones(1,N+1)); %distancia entre vehiculo 1 y 2

a1 = binvar(1,nv);      a2 = binvar(1,nv);      a3 = binvar(1,nv);      a4 = binvar(1,nv);
g1 = binvar(1,nv);      g2 = binvar(1,nv);      g3 = binvar(1,nv);      g4 = binvar(1,nv);
Aa1 = binvar(1,nv);     Aa2 = binvar(1,nv);     Aa3 = binvar(1,nv);     Aa4 = binvar(1,nv);
Bb1 = binvar(1,nv);     Bb2 = binvar(1,nv);     Bb3 = binvar(1,nv);     Bb4 = binvar(1,nv);
Gg1 = binvar(1,nv);     Gg2 = binvar(1,nv);     Gg3 = binvar(1,nv);     Gg4 = binvar(1,nv);
Ss1 = binvar(1,nv);     Ss2 = binvar(1,nv);     Ss3 = binvar(1,nv);     Ss4 = binvar(1,nv);
Nn1 = binvar(1,nv);     Nn2 = binvar(1,nv);     Nn3 = binvar(1,nv);     Nn4 = binvar(1,nv);
b1 = binvar(1,nv);      b2 = binvar(1,nv);      b3 = binvar(1,nv);      b4 = binvar(1,nv);
z1 = binvar(1,nv);      z2 = binvar(1,nv);      z3 = binvar(1,nv);      z4 = binvar(1,nv);
n1 = binvar(1,nv);      n2 = binvar(1,nv);      n3 = binvar(1,nv);      n4 = binvar(1,nv);

D1 = binvar(3,1,nv);    D2 = binvar(3,1,nv);    D3 = binvar(3,1,nv);    D4 = binvar(3,1,nv);
G1 = binvar(3,1,nv);    G2 = binvar(3,1,nv);    G3 = binvar(3,1,nv);    G4 = binvar(3,1,nv);
B1 = binvar(2,1,nv);    B2 = binvar(2,1,nv);    B3 = binvar(2,1,nv);    B4 = binvar(2,1,nv);
Z1 = binvar(5,1,nv);    Z2 = binvar(5,1,nv);    Z3 = binvar(5,1,nv);    Z4 = binvar(5,1,nv);
N1 = binvar(3,1,nv);    N2 = binvar(3,1,nv);    N3 = binvar(3,1,nv);    N4 = binvar(3,1,nv);

p_a = sdpvar(1);
p_z = sdpvar(1);

constraints = [-0.8 <= diff([p_a a{:}]) <= 0.8];
constraints = [constraints, -1 <= diff([p_z z{:}]) <= 1];
objective   = 0;
i=1;
%-----creacion de funcion objetivo y restricciones--------------------
for k = 1:N
 objective = objective+(v{k}-Vd)'*Q*(v{k}-Vd) + (z{k}-Zd)'*R*(z{k}-Zd); % calculate obj
  
  % Feasible region
    constraints = [constraints ,1<=    z{k}     <= L,
                               1 <=    z_2      <= L,%tome valores posibles
                               1 <=    z_3      <= L,%tome valores posibles
                                0<=    v{k+1}   <= V_max,%no exceda las velocidades 
                         z{k}-[1]<=    z{k+1}   <=z{k}+[1],
                           -A_max<=    a{k}     <= A_max];%paso de un carril
                             
    constraints = [constraints, v{k+1} == v{k}+T*a{k}];%velocidad futura
  
    for i=1:nv
% ------------------------------------vehiculo 1-------------------------------    
%------------------si dz=0  -------------------->>>    dij>= Ds----------------

    constraints = [constraints, -10000  <=  dis12{k+1}(i) <= 100000,
                                  mmin  <= z_2(i)-z{k+1}  <= Mmax];
    constraints = [constraints, dis12{k+1}(i) == dis12{k}(i)+T*(v_2(i)-v{k})];
%.........................alpha...............................
constraints = [constraints, [D1(1,1,i)+D1(2,1,i)+D1(3,1,i)==1], 
              implies( D1(1,1,i), [ a1(i)==0, z_2(i)-z{k} <=-0.2 ]);
              implies( D1(2,1,i), [ a1(i)==1, -0.2<=z_2(i)-z{k} <=0.2 ]);
              implies( D1(3,1,i), [ a1(i)==0, 0.2 <= z_2(i)-z{k} ]) ];
%.........................Beta...............................
constraints = [constraints, [sum(B1,1,i)==1], 
              implies(B1(1,1,i),[ b1(i)==1, dis12{1}(i) >=0 ]);
              implies(B1(2,1,i),[ b1(i)==0, dis12{1}(i) <=0 ]) ];
          
constraints = [constraints, [dis12{1}(i)<=100000]]; 

% %.........................Gamma...............................
constraints = [constraints, [G1(1,1,i)+G1(2,1,i)+G1(3,1,i)==1], 
              implies( G1(1,1,i), [ g1(i)==0, z_2(i)-z{k+1} <=-0.1 ]);
              implies( G1(2,1,i), [ g1(i)==1, -0.1<=z_2(i)-z{k+1} <=0.2 ]);
              implies( G1(3,1,i), [ g1(i)==0, 0.1 <= z_2(i)-z{k+1} ]) ];   
          
constraints = [constraints, [1<=z{k+1}<=L]];           

% %.........................Lateral distance...............................
constraints = [constraints, [sum(Z1,1,i)==1], 
              implies( Z1(1,1,i), [ z1(i)==0,       z_2(i)-z{k} <= -1.1 ]);
              implies( Z1(2,1,i), [ z1(i)==1, -1.1<=z_2(i)-z{k} <= -0.9 ]);
              implies( Z1(3,1,i), [ z1(i)==0, -0.9<=z_2(i)-z{k} <= 0.9 ]);
              implies( Z1(4,1,i), [ z1(i)==1,  0.9<=z_2(i)-z{k} <= 1.1 ]);
              implies( Z1(5,1,i), [ z1(i)==0,            1.1 <= z_2(i)-z{k} ]) ];   
constraints = [constraints, [1<=p_z<=L]];

% %.........................lateral safety distance...............................
constraints = [constraints, [sum(N1,1,i)==1], 
              implies( N1(1,1,i), [ n1(i)==0,       dis12{1}(i) <= -Dl]);
              implies( N1(2,1,i), [ n1(i)==1, -Dl<= dis12{1}(i) <= Dl ]);
              implies( N1(3,1,i), [ n1(i)==0,  Dl<= dis12{1}(i)]) ];   
%................................................................................
constraints = [constraints,  Aa1(i)*(Bb1(i)*(Ds - dis12{k+1}(i))+(1-Bb1(i))*(Ds + dis12{k+1}(i)))<=0];
constraints = [constraints,  Aa1(i)*Gg1(i)*(-Bb1(i)*(T*(v_2(i)-v{k})+dis12{k}(i))+(1-Bb1(i))*(T*(v_2(i)-v{k})+dis12{k}(i)))<=0];
constraints = [constraints, Ss1(i)*(Nn1(i))*(z{k}-p_z)==0];

% ------------------------------------vehiculo 2-------------------------------    
%------------------si dz=0  -------------------->>>    dij>= Ds----------------

    constraints = [constraints, -10000  <=  dis13{k+1}(i) <= 100000,
                                  mmin  <= z_3(i)-z{k+1}  <= Mmax];
    constraints = [constraints, dis13{k+1}(i) == dis13{k}(i)+T*(v_2(i)-v{k})];
%.........................alpha...............................
constraints = [constraints, [D2(1,1,i)+D2(2,1,i)+D2(3,1,i)==1], 
              implies( D2(1,1,i), [ a2(i)==0, z_3(i)-z{k} <=-0.2 ]);
              implies( D2(2,1,i), [ a2(i)==1, -0.2<=z_3(i)-z{k} <=0.2 ]);
              implies( D2(3,1,i), [ a2(i)==0, 0.2 <= z_3(i)-z{k} ]) ];
%.........................Beta...............................
constraints = [constraints, [sum(B2,1,i)==1], 
              implies(B2(1,1,i),[ b2(i)==1, dis13{1}(i) >=0 ]);
              implies(B2(2,1,i),[ b2(i)==0, dis13{1}(i) <=0 ]) ];
          
constraints = [constraints, [dis13{1}(i)<=100000]]; 

% %.........................Gamma...............................
constraints = [constraints, [G2(1,1,i)+G2(2,1,i)+G2(3,1,i)==1], 
              implies( G2(1,1,i), [ g2(i)==0, z_3(i)-z{k+1} <=-0.1 ]);
              implies( G2(2,1,i), [ g2(i)==1, -0.1<=z_3(i)-z{k+1} <=0.2 ]);
              implies( G2(3,1,i), [ g2(i)==0, 0.1 <= z_3(i)-z{k+1} ]) ];   
          
constraints = [constraints, [1<=z{k+1}<=L]];           

% %.........................Lateral distance...............................
constraints = [constraints, [sum(Z2,1,i)==1], 
              implies( Z2(1,1,i), [ z2(i)==0,       z_3(i)-z{k} <= -1.1 ]);
              implies( Z2(2,1,i), [ z2(i)==1, -1.1<=z_3(i)-z{k} <= -0.9 ]);
              implies( Z2(3,1,i), [ z2(i)==0, -0.9<=z_3(i)-z{k} <= 0.9 ]);
              implies( Z2(4,1,i), [ z2(i)==1,  0.9<=z_3(i)-z{k} <= 1.1 ]);
              implies( Z2(5,1,i), [ z2(i)==0,            1.1 <= z_3(i)-z{k} ]) ];   
constraints = [constraints, [1<=p_z<=L]];

% %.........................lateral safety distance...............................
constraints = [constraints, [sum(N2,1,i)==1], 
              implies( N2(1,1,i), [ n2(i)==0,       dis13{1}(i) <= -Dl]);
              implies( N2(2,1,i), [ n2(i)==1, -Dl<= dis13{1}(i) <= Dl ]);
              implies( N2(3,1,i), [ n2(i)==0,  Dl<= dis13{1}(i)]) ];   
%................................................................................
constraints = [constraints,  Aa2(i)*(Bb2(i)*(Ds - dis13{k+1}(i))+(1-Bb2(i))*(Ds + dis13{k+1}(i)))<=0];
constraints = [constraints,  Aa2(i)*Gg2(i)*(-Bb2(i)*(T*(v_2(i)-v{k})+dis13{k}(i))+(1-Bb2(i))*(T*(v_2(i)-v{k})+dis13{k}(i)))<=0];
constraints = [constraints, Ss2(i)*(Nn2(i))*(z{k}-p_z)==0];

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
    end
end
objective = objective+(v{N+1}-Vd)'*Q*(v{N+1}-Vd) + (z{N+1}-Zd)'*R*(z{N+1}-Zd); % calculate obj
%% solver definition   

parameters_in = {Vd,Zd,v{1},p_a,p_z,...
                            v_2,z_2,dis12{1},Aa1,Bb1,Gg1,Ss1,Nn1,...
                            v_3,z_3,dis13{1},Aa2,Bb2,Gg2,Ss2,Nn2};
solutions_out = {[a{:}], [z{:}], [v{:}], [a1], [dis12{:}], [b1], [g1], [z1], [n1]...
                                         [a2], [dis13{:}], [b2], [g2], [z2], [n2]};

controller1 = optimizer(constraints, objective,sdpsettings('solver','gurobi'),parameters_in,solutions_out);
%------condiciones iniciales----------
vel=[15; 15; 10; 20];% velociodad inicial
zel=[5; 2; 1; 3]; %carril inicial
zel2=[zel(2); zel(1); zel(3); zel(4)]; %carril inicial
zel3=[zel(3); zel(1); zel(2); zel(4)]; %carril inicial
zel4=[zel(4); zel(1); zel(2); zel(3)]; %carril inicial
Vdes=[5; 20; 25; 10]; %velocidad deseada
Zdes=[1; 4; 3; 5];
%---distancia inicial de cada agente
% disij= Zj-zi
d1i = [10; 0; 0];
d2i = [-d1i(1); -d1i(1)+d1i(2); -d1i(1)+d1i(3)];
d3i = [-d1i(2); -d1i(2)+d1i(1); -d1i(2)+d1i(3)];
past_a=[0 0 0]';

%define las condiciones iniciales que deben tener las variables
%logicas
 %........vehiculo 1....................vehiculo 2...........
 alogic1_1=[0]; alogic2_1=[0];   alogic1_2=[0]; alogic2_2=[0];
 blogic1_1=[1]; blogic2_1=[0];   blogic1_2=[1]; blogic2_2=[0];
 G1logic_1=[1]; G2logic_1=[1];   G1logic_2=[1]; G2logic_2=[1];
 S1logic_1=[1]; S2logic_1=[1];   S1logic_2=[1]; S2logic_2=[1];
 N1logic_1=[1]; N2logic_1=[1];   N1logic_2=[1]; N2logic_2=[1];
%........vehiculo 3....................vehiculo 4...........
 alogic1_3=[0]; alogic2_3=[0];   alogic1_4=[0]; alogic2_4=[0];
 blogic1_3=[1]; blogic2_3=[0];   blogic1_4=[1]; blogic2_4=[0];
 G1logic_3=[1]; G2logic_3=[1];   G1logic_4=[1]; G2logic_4=[1];
 S1logic_3=[1]; S2logic_3=[1];   S1logic_4=[1]; S2logic_4=[1];
 N1logic_3=[1]; N2logic_3=[1];   N1logic_4=[1]; N2logic_4=[1];

def_condinicial(zel,  d1i,Dl,alogic1_1,alogic2_1,blogic1_1,blogic2_1,G1logic_1,G2logic_1,S1logic_1,S2logic_1,N1logic_1,N2logic_1);
def_condinicial(zel2, d2i,Dl,alogic1_2,alogic2_2,blogic1_2,blogic2_2,G1logic_2,G2logic_2,S1logic_2,S2logic_2,N1logic_2,N2logic_2);
def_condinicial(zel3, d3i,Dl,alogic1_3,alogic2_3,blogic1_3,blogic2_3,G1logic_3,G2logic_3,S1logic_3,S2logic_3,N1logic_3,N2logic_3);
def_condinicial(zel4, d4i,Dl,alogic1_4,alogic2_4,blogic1_4,blogic2_4,G1logic_4,G2logic_4,S1logic_4,S2logic_4,N1logic_4,N2logic_4);


% hold on
vhist = vel;
zhist = zel;
ahist = past_a;
dhist = d1i;



%
% ...historial variables logicas
 A1hist =[0]'; A2hist =[0]';
 b1hist=[]; b2hist=[];
 g1hist=[];  g2hist=[];
 n1hist=[];  n2hist=[];
% ..........historial de las predicciones 
 vp1hist=[]; vp2hist=[]; 
 zp1hist=[]; zp2hist=[];
 z1hist=[]; z2hist=[];
for i = 1:30
% parameters_in = {Vd,Zd,v{1},p_a,p_z,...
%                             v_2,z_2,dis12{1},Aa1,Bb1,Gg1,Ss1,Nn1,...
%                             v_3,z_3,dis13{1},Aa2,Bb2,Gg2,Ss2,Nn2};
% solutions_out = {[a{:}], [z{:}], [v{:}], [a1], [dis12{:}], [b1], [g1], [z1], [n1]...
%                                          [a2], [dis13{:}], [b2], [g2], [z2], [n2]};
    inputs = {Vdes(1),Zdes(1),vel(1),past_a(1),zel(1),...
                        vel(2),zel(2),d1i(1),alogic1_1,blogic1_1,G1logic_1,S1logic_1,N1logic_1,...
                        vel(3),zel(3),d1i(2),alogic2_1,blogic2_1,G2logic_1,S2logic_1,N2logic_1};
    [solutions1,diagnostics] = controller1{inputs};    
     
    A = solutions1{1};past_a(1) = A(:,1);
    Z = solutions1{2};   zel(1)=Z(:,1);          zp1hist=[zp1hist; Z];
    V = solutions1{3};   vp1hist=[vp1hist; V];
    A1 = solutions1{4};                          alogic1_1=A1(:,1);
    g1 = solutions1{5};  %g1hist=[g1hist g1];
    B1 = solutions1{6};  b1hist=[b1hist B1];     blogic1_1=B1(:,1);
    Gg1 = solutions1{7}; g1hist=[g1hist Gg1];    G1logic_1=Gg1(:,1);
    Z1 = solutions1{8};  z1hist=[z1hist Z1];     S1logic_1=Z1(:,1);
    N1 = solutions1{9};  n1hist=[n1hist N1];     N1logic_1=N1(:,1);
    A2 = solutions1{10};                         alogic2_1=A2(:,1);
    g2 = solutions1{11};  g2hist=[g2hist g2];
    B2 = solutions1{12};  b2hist=[b2hist B2];    blogic2_1=B2(:,1);
    Gg2 = solutions1{13}; g2hist=[g2hist Gg2];   G2logic_1=Gg2(:,1);
    Z2 = solutions1{14};  z2hist=[z2hist Z2];    S2logic_1=Z2(:,1);
    N2 = solutions1{15};  n2hist=[n2hist N2];    N2logic_1=N2(:,1);    
    
    A1hist =[A1hist A1];    
    
        
    if diagnostics == 1
        error('you are close, keep trying 1');
    end   
    
%.........................solver vehiculo 2............................
inputs2 = {Vdes(2),Zdes(2),vel(2),past_a(2),zel(2),...
                        vel(1),zel(1),d2i(1),alogic1_2,blogic1_2,G1logic_2,S1logic_2,N1logic_2,...
                        vel(3),zel(3),d2i(2),alogic2_2,blogic2_2,G2logic_2,S2logic_2,N2logic_2};
    [solutions2,diagnostics2] = controller1{inputs2};           

 
    
    A = solutions2{1};past_a(2) = A(:,1);
    Z = solutions2{2};   zel(2)=Z(:,1);          zp2hist=[zp2hist; Z];
    V = solutions2{3};   vp2hist=[vp2hist; V];
    A1 = solutions2{4};                         alogic1_2=A1(:,1);
    g1 = solutions2{5};  g1hist_1=[g1hist g1];
    B1 = solutions2{6};  b1hist_1=[b1hist B1];  blogic1_2=B1(:,1);
    Gg1 = solutions2{7}; g1hist_1=[g1hist Gg1]; G1logic_2=Gg1(:,1);
    Z1 = solutions2{8};  z1hist=[z1hist Z1];    S1logic_2=Z1(:,1);
    N1 = solutions2{9};  n1hist=[n1hist N1];    N1logic_2=N1(:,1);
    A2 = solutions2{10};                        alogic2_2=A2(:,1);
    g2 = solutions2{11};  g2hist=[g2hist g2];
    B2 = solutions2{12};  b2hist=[b2hist B2];   blogic2_2=B2(:,1);
    Gg2 = solutions2{13}; g2hist=[g2hist Gg2];  G2logic_2=Gg2(:,1);
    Z2 = solutions2{14};  z2hist=[z2hist Z2];   S2logic_2=Z2(:,1);
    N2 = solutions2{15};  n2hist=[n2hist N2];   N2logic_2=N2(:,1); 
   
%     A2hist =[A2hist A2];    
    d2i = d2i+T*(vel(2:(2+1))-ones(2,1)*vel(1));
   
    if diagnostics2 == 1
        error('you are close, keep trying 2');
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
    
    vel = vel+T*past_a;
    vhist = [vhist vel];
    zhist = [zhist zel];
    ahist = [ahist past_a];
    dhist = [dhist d1i];

    A2hist =[A2hist A2];
  
%     pause(0.05)   

end



Draw_MIPG(vhist,zhist,...
                            vp1hist,zp1hist,...
                            vp2hist,zp2hist,...
                                                dhist,T,N)

