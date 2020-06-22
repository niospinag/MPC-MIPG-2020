% point stabilization + Multiple shooting
clear all
close all
clc

% CasADi v3.4.5
addpath('C:\Users\Personal\Desktop\casadi-windows-matlabR2016a-v3.4.5')
import casadi.*

T = 0.2; %[s]
N = 10; % prediction horizon
rob_diam = 0.3;

v_max = 0.6; v_min = -v_max;
omega_max = pi/4; omega_min = -omega_max;

x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
states = [x;y;theta]; n_states = length(states);

v = SX.sym('v'); omega = SX.sym('omega');
controls = [v;omega]; n_controls = length(controls);
rhs = [v*cos(theta);v*sin(theta);omega]; % system r.h.s
%-------------------definicion de variables---------------------

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U1 = SX.sym('U1',n_controls,N); % Decision variables (controls)
P1 = SX.sym('P1',2*(n_states + n_states));
X1 = SX.sym('X1',n_states,(N+1));

U2 = SX.sym('U2',n_controls,N); % Decision variables (controls)

X2 = SX.sym('X2',n_states,(N+1));

%---------------------funcion objetivo -----------------------------

Q = zeros(3,3); Q(1,1) = 1;Q(2,2) = 5;Q(3,3) = 0.1; % weighing matrices (states)
R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05; % weighing matrices (controls)

%-----------agent 1----------

obj1 = 0; % Objective function
g1 = [];  % constraints vector

st1  = X1(:,1); % initial state
g1 = [g1;st1-P1(1:3)]; % initial condition constraints
for k = 1:N
    st1 = X1(:,k);  con1 = U1(:,k);
    obj1 = obj1+(st1-P1(4:6))'*Q*(st1-P1(4:6)) + con1'*R*con1; % calculate obj
    st_next1 = X1(:,k+1);
    f_value1 = f(st1,con1);
    st_next_euler1 = st1+ (T*f_value1);
    g1 = [g1;st_next1-st_next_euler1]; % compute constraints
end

%-----------agent 2----------

obj2 = 0; % Objective function
g2 = [];  % constraints vector

st2  = X2(:,1); % initial state
g2 = [g2;st2-P1(7:9)]; % initial condition constraints
for k = 1:N
    st2 = X2(:,k);  con2 = U2(:,k);
    obj2 = obj2+(st2-P1(10:12))'*Q*(st2-P1(10:12)) + con2'*R*con2; % calculate obj
    st_next2 = X2(:,k+1);
    f_value2 = f(st2,con2);
    st_next_euler2 = st2+ (T*f_value2);
    g2 = [g2;st_next2-st_next_euler2]; % compute constraints
end

%----------------------constraints for obstacle------------
obs_x = 1.5; % meters 1.5
obs_y = 0.7; % meters 0.7
obs_diam = 1; % meters 1

%-----------agent 1----------

for k = 1:N+1   % box constraints due to the map margins
    g1 = [g1 ; -sqrt((X1(1,k)-obs_x)^2+(X1(2,k)-obs_y)^2) + (rob_diam/2 + obs_diam/2)];
end

%-----------agent 2----------


for k = 1:N+1   % box constraints due to the map margins
    g2 = [g2 ; -sqrt((X2(1,k)-obs_x)^2+(X2(2,k)-obs_y)^2) + (rob_diam/2 + obs_diam/2)];
    
end



%----------------------constraints for collition------------
%-----------agent 1----------

for k = 1:N+1   % box constraints due to the map margins
    %g1 = [g1 ; -sqrt((X1(1,k)-obs_x)^2+(X1(2,k)-obs_y)^2) + (rob_diam/2 + obs_diam/2)];
    g1 = [g1 ; -sqrt((X1(1,k)-P1(7))^2+(X1(2,k)-P1(8))^2) + (rob_diam/2 + rob_diam/2)];
end

% make the decision variable one column  vector
OPT_variables1 = [reshape(X1,3*(N+1),1);reshape(U1,2*N,1)];

nlp_prob1 = struct('f', obj1, 'x', OPT_variables1, 'g', g1, 'p', P1);

%-----------agent 2----------


for k = 1:N+1   % box constraints due to the map margins
    g2 = [g2 ; -sqrt((X2(1,k)-P1(1))^2+(X2(2,k)-P1(2))^2) + (rob_diam/2 + rob_diam/2)];
    %g1 = [g1 ; -sqrt((X1(1,k)-P1(7))^2+(X1(2,k)-P1(8))^2) + (rob_diam/2 + obs_diam/2)];
end

OPT_variables2 = [reshape(X2,3*(N+1),1);reshape(U2,2*N,1)];

nlp_prob2 = struct('f', obj2, 'x', OPT_variables2, 'g', g2, 'p', P1);
%--------------------------------------------------------------------------
% 
% OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];
% nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);
% 
% opts = struct;
% opts.ipopt.max_iter = 100;
% opts.ipopt.print_level =0;%0,3
% opts.print_time = 0;
% opts.ipopt.acceptable_tol =1e-8;
% opts.ipopt.acceptable_obj_change_tol = 1e-6;
% 
% solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

%------------------caracteristicas del solver------------------------------



opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;


%-----------agente 1------------------
solver1 = nlpsol('solver', 'ipopt', nlp_prob1,opts);


args1 = struct;

args1.lbg(1:3*(N+1)) = 0;  % -1e-20  % Equality constraints
args1.ubg(1:3*(N+1)) = 0;  % 1e-20   % Equality constraints

args1.lbg(3*(N+1)+1 : 4*(N+1)+ (N+1)) = -inf; % inequality constraints<---------------------
args1.ubg(3*(N+1)+1 : 4*(N+1)+ (N+1)) = 0; % inequality constraints<---------------------

args1.lbx(1:3:3*(N+1),1) = -2; %state x lower bound
args1.ubx(1:3:3*(N+1),1) = 10; %state x upper bound  <----------------------
args1.lbx(2:3:3*(N+1),1) = -2; %state y lower bound
args1.ubx(2:3:3*(N+1),1) = 2; %state y upper bound
args1.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args1.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

args1.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args1.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args1.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
args1.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound


%-------------agente 2---------------------
solver2 = nlpsol('solver', 'ipopt', nlp_prob2,opts);

args2 = struct;

args2.lbg(1:3*(N+1)) = 0;  % -1e-20  % Equality constraints
args2.ubg(1:3*(N+1)) = 0;  % 1e-20   % Equality constraints

args2.lbg(3*(N+1)+1 : 4*(N+1)+ (N+1)) = -inf; % inequality constraints<---------------------
args2.ubg(3*(N+1)+1 : 4*(N+1)+ (N+1)) = 0; % inequality constraints<---------------------

args2.lbx(1:3:3*(N+1),1) = -2; %state x lower bound
args2.ubx(1:3:3*(N+1),1) = 10; %state x upper bound  <----------------------
args2.lbx(2:3:3*(N+1),1) = -2; %state y lower bound
args2.ubx(2:3:3*(N+1),1) = 2; %state y upper bound
args2.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args2.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

args2.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args2.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args2.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
args2.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound

%----------------------------------------------

% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;
t1 = 0;

t(1) = t0;
sim_tim = 30; % Maximum simulation time
mpciter = 0;
%---------------------------agente 1-------------------------------

x0_1 = [0 ; 0.5 ; 0.0];    % initial condition.
xs_1 = [6.5 ; 1.5 ; 0.0]; % Reference posture.

xx_1(:,1) = x0_1; % xx contains the history of states

u0_1 = zeros(N,2);  % two control inputs for each robot
X0_1 = repmat(x0_1,1,N+1)'; % initialization of the states decision variables

% Start MPC
xx1_1 = [];%history of states
u_cl_1=[];%hisory of controls

%---------------------------agente 2-------------------------------

x0_2 = [0 ; 1.5 ; 0.0];    % initial condition.
xs_2 = [5 ; 0 ; 0.0]; % Reference posture.

xx_2(:,1) = x0_2; % xx contains the history of states

u0_2 = zeros(N,2);  % two control inputs for each robot
X0_2 = repmat(x0_2,1,N+1)'; % initialization of the states decision variables

% Start MPC
xx1_2 = [];%history of states
u_cl_2=[];%hisory of controls


% the main simulaton loop... it works as long as the error is greater
% than 10^-6 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
while(norm((x0_1-xs_1),2) > 1e-2 && norm((x0_2-xs_2),2) > 1e-2 && mpciter < sim_tim / T)
    args1.p   = [x0_1;xs_1;x0_2;xs_2]; % set the values of the parameters vector
    args2.p   = [x0_1;xs_1;x0_2;xs_2]; % set the values of the parameters vector
    
    % initial value of the optimization variables
    %-------agent1
    args1.x0  = [reshape(X0_1',3*(N+1),1);reshape(u0_1',2*N,1)];
    sol1 = solver1('x0', args1.x0, 'lbx', args1.lbx, 'ubx', args1.ubx,...
        'lbg', args1.lbg, 'ubg', args1.ubg,'p',args1.p);
    u_1 = reshape(full(sol1.x(3*(N+1)+1:end))',2,N)'; % get controls only from the solution
    xx1_1(:,1:3,mpciter+1)= reshape(full(sol1.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    u_cl_1= [u_cl_1 ; u_1(1,:)];
    
    %-------agent2
    args2.x0  = [reshape(X0_2',3*(N+1),1);reshape(u0_2',2*N,1)];
    sol2 = solver2('x0', args2.x0, 'lbx', args2.lbx, 'ubx', args2.ubx,...
        'lbg', args2.lbg, 'ubg', args2.ubg,'p',args2.p);
    u_2 = reshape(full(sol2.x(3*(N+1)+1:end))',2,N)'; % get controls only from the solution
    xx1_2(:,1:3,mpciter+1)= reshape(full(sol2.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    u_cl_2= [u_cl_2 ; u_2(1,:)];
    
    
    t(mpciter+1) = t0;
    % Apply the control and shift the solution
    [t0, x0_1, u0_1] = shift(T, t0, x0_1, u_1,f);
    [t1, x0_2, u0_2] = shift(T, t0, x0_2, u_2,f);
    
    %--------agente1    
    xx_1(:,mpciter+2) = x0_1;
    X0_1 = reshape(full(sol1.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0_1 = [X0_1(2:end,:);X0_1(end,:)];
    
   %--------agente2    
    xx_2(:,mpciter+2) = x0_2;
    X0_2 = reshape(full(sol2.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0_2 = [X0_2(2:end,:);X0_2(end,:)];
    
    mpciter
    mpciter = mpciter + 1;
end;
main_loop_time = toc(main_loop);
ss_error = norm((x0_1-xs_1),2)
average_mpc_time = main_loop_time/(mpciter+1)

Draw_MPC_point_stabilization_v1 (t,xx_1,xx1_1,u_cl_1,xs_1,xx_2,xx1_2,u_cl_2,xs_2,N,rob_diam,obs_x,obs_y,obs_diam)
%Draw_MPC_point_stabilization_v1 (t,xx_2,xx1_2,u_cl_2,xs_2,N,rob_diam)

