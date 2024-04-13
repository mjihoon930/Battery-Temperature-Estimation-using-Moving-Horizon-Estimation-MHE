%%  Project: Temperature and Generated Heat Estimation using Moving Horizon Estimation (MHE)
%   Date: 03/14/2024
%
%   Memeber: Nitisha Ahuja, Jihoon Moon
% 
%   Pennsylvania State University
%   Mechanical Engineering
%   Mechatronics Research Laboratory
%   https://www.me.psu.edu/mrl/

clear all
close all
clc

%% Import CasADi Package

% Window
%addpath('C:/Users/jkm6638/OneDrive - The Pennsylvania State University/Research/Code/Rsc Estimation/Rsc Estimation with MHE/casadi-3.6.3-windows64-matlab2018b')

% Mac
addpath('/Users/moonjihoon/Library/CloudStorage/OneDrive-ThePennsylvaniaStateUniversity/Research/Code/Batttery T and Heat Estimation using MHE/casadi-3')

import casadi.*

%% Input Profile
[file, path] = uigetfile('*.csv', ' Please choose file');
filename = [path, file];
Data_table = readtable(filename);

%% Extract Data

Data_Type = 'Cold';

time1      = Data_table(:,1).Time_s_;
[Index,~]  = find((time1./3600)>0);
time1      = Data_table(Index,1).Time_s_;
time1      = time1-time1(1);
curr1      = Data_table(Index,2).Current_mA_;
curr1      = curr1./1000; % Amp
volt1      = Data_table(Index,3).Voltage_V_;
Temp1      = Data_table(Index,4).Temperature_C_;
Cycle_no1  = Data_table.CycleNumber;
Cycle_no1  = Cycle_no1(Index);
dt         = 10;

switch(Data_Type)

    case 'Normal'

        Tamb       = 25.*ones(length(curr1),1);

        % State Limits Variables
        Charge_current = 4.85;
        R0_1st_cycle   = 0.061;%Res_HPPC(:,1);
        
        Q_min          = 0;                                                         % Minimum Generated Heat
        Q_max          = 2*R0_1st_cycle*Charge_current^2;                           % Maximum Generated Heat
        
        T_min          = 10;                                                        % Minimum Surface Temperature
        T_max          = 50;                                                        % Maximum Surface Temperature
        
        Tamb_min       = 20;                                                        % Minimum Ambient Temperature
        Tamb_max       = 30;                                                        % Maximum Ambient Temperature

        load('Res_HPPC1.mat');
    
    case 'Cold'

        Tamb       = -3.*ones(length(curr1),1);

        % State Limits Variables
        Charge_current = 4.85;
        R0_1st_cycle   = 0.0193021525964750;
        
        Q_min          = 0;                                                         % Minimum Generated Heat
        Q_max          = 2;%2*R0_1st_cycle*Charge_current^2;                           % Maximum Generated Heat
        
        T_min          = -10;                                                        % Minimum Surface Temperature
        T_max          = 10;                                                        % Maximum Surface Temperature
        
        Tamb_min       = -6;                                                        % Minimum Ambient Temperature
        Tamb_max       = 0;                                                        % Maximum Ambient Temperature

        load('Res_HPPC.mat');

end

%% Resampling the data for equal intervals
Te11       = timeseries([volt1,curr1,Temp1, Tamb,Cycle_no1],time1-time1(1));
t11        = [time1(1)-time1(1):dt:time1(end)-time1(1)];
dt         = t11(2)-t11(1);
Te2        = resample(Te11,t11');
R_Voltage1 = Te2.Data(:,1);
R_Current1 = Te2.Data(:,2);
R_Temper1  = Te2.Data(:,3);
R_Tamb1    = Te2.Data(:,4);
R_Cycle_no = Te2.Data(:,5);
sim_tim    = length(t11);

p = polyfit(Res_HPPC(:,1), Res_HPPC(:,2), 1);
R_Res = polyval(p, R_Cycle_no );

%% Set Horizon Length
N_MHE = 10;     

%% System Dynamics

load('theta.mat')

% Define State Space Model
A                 = [-theta(1),theta(2);0,0];
B                 = [theta(1);0];
C                 = [1,0];
D                 = [0];

% Continuous Time System Dynamics
Orig_sys          = ss(A,B,C,D);

A_continuous      = Orig_sys.A;
B_continuous      = Orig_sys.B;
C_continuous      = Orig_sys.C;
D_continuous      = Orig_sys.D;

% Discrete Time System Dynamics
Discrete_orig_Sys = c2d(Orig_sys,dt);    

A_discrete = Discrete_orig_Sys.A;
B_discrete = Discrete_orig_Sys.B;
C_discrete = Discrete_orig_Sys.C;
D_discrete = Discrete_orig_Sys.D;

%% States
Ti       = SX.sym('Ti');                                        % State 1: Surface Temperature
Q_h      = SX.sym('Q_h');                                       % State 2: Generated Heat

states   = [Ti; Q_h];                                           % State Vector
n_states = length(states);                                      % Number of States

%% Control
T_amb      = SX.sym('T_amb');                                   % Control Input: Ambient Temperature

controls   = [T_amb];                                           % Control Input
n_controls = length(controls);                                  % Number of Control Input

%% Disturbances
w1 = SX.sym('w1');                                              % State 1 Process Noise
w2 = SX.sym('w2');                                              % State 2 Process Noise

disturbances   = [w1 ; w2];                                     % Disturbances Vector
n_disturbances = length(disturbances);                          % Number of Disturbances

%% Covariance Matrix
Meas_noise_cov  = 1;                                            % Measuremet Noise Covariance Matrix Value
Meas_cov        = diag(Meas_noise_cov).^2;                      % Measuremet Noise Covariance Matrix

State_noise_cov = 1000;                                         % Process Noise Covariance Matrix Value
State_cov       = diag([State_noise_cov,State_noise_cov]).^2;   % Process Noise Covariance Matrix

%% State Dynamics
rhs = A_continuous*[Ti; Q_h] + B_continuous*T_amb;              % System Right Hand Side
f   = Function('f',{states,controls,disturbances},{rhs});       % State Dynamics Function

measurement_rhs = Ti;
h = Function('h',{states,controls},{measurement_rhs});          % Measurement Function

%% Decision Variables
X  = SX.sym('X',n_states, (N_MHE+1));                           % Decision Variables for State
Wj = SX.sym('Wj',n_disturbances, (N_MHE));                      % Decision Variables for Process Noise

%% Parameters to be Passed: Measuremnts and control inputs
P  = SX.sym('P',1, (N_MHE+1)+ (N_MHE+1) + n_states*n_states + n_states);                       % first N_MHE+a correspond to surface temperature measurments and second N_MHE columns Correspond to Control inputs which are known

%% Weighting Matrices
V = inv(sqrt(Meas_cov));                                        % Weighting Matrix for Measurement
W = inv(sqrt(State_cov));                                       % Weighting Matrix for State

%% Define Ojective Function

obj = 0; % Objective function
g   = [];  % constraints vector

% Arrival Cost
first_state = X(:,1);
first_state_prior = reshape(P(:,(end-n_states+1):end),n_states,1);

P_arrival = inv(reshape(P(:,((2*N_MHE+2)+1):((2*N_MHE+2))+(n_states*n_states)),n_states,n_states));

obj = obj + ( (first_state - first_state_prior)' * P_arrival * (first_state - first_state_prior) );

% Output Surface Temperature
for k = 1:N_MHE+1

    st      = X(:,k);
    con     = P(:,(N_MHE+1+k));
    h_x     = h(st,con);
    y_tilde = P(:,k);
    
    obj     = obj + (y_tilde - h_x)' * V * (y_tilde - h_x);     % calculate obj based on difference between measured temperature and measurment model

end

% State or Process Noise
for k = 1:N_MHE

    st_pr  = X(:,k);
    st_po  = X(:,k+1);
    con    = P(:, (N_MHE+1+k));
    dist   = Wj(:,k);
    
    obj = obj + ((st_po - (A_discrete*st_pr + B_discrete*con + dist))' * W * (st_po - (A_discrete*st_pr + B_discrete*con + dist)));

end

% Multiple Shooting Constraints
for k = 1:N_MHE
    
    st            = X(:,k);  
    con           = P(:,(N_MHE+1+k));
    dist          = Wj(:,k);
    st_next       = X(:,k+1);
    f_value       = f(st,con,dist);
    st_next_euler = st + (dt*f_value);

    g = [g;st_next-st_next_euler]; % compute constraints

end

%% MHE solver
% Make the Decision Variable One Column  Vector
OPT_variables = [reshape(X,n_states*(N_MHE+1),1) ; reshape(Wj,n_disturbances*(N_MHE),1)];

% Structure
nlp_mhe       = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

% Option
opts                                  = struct;
opts.ipopt.max_iter                   = 4000;
opts.ipopt.print_level                = 0;
opts.print_time                       = 0;
opts.ipopt.acceptable_tol             = 1e-10;
opts.ipopt.acceptable_obj_change_tol  = 1e-8;

% Solver
solver = nlpsol('solver', 'ipopt', nlp_mhe,opts);

args   = struct;

%% Constraint 

num_con = n_states + n_disturbances;

% equality constraints
args.lbg(1:n_states*(N_MHE)) = 0;
args.ubg(1:n_states*(N_MHE)) = 0;

% State constraints
args.lbx(1:num_con:(n_states*(N_MHE+1) + n_disturbances*N_MHE),1) = T_min;  % Surface T Lower Bound
args.ubx(1:num_con:(n_states*(N_MHE+1) + n_disturbances*N_MHE),1) = T_max;  % Surface T Upper Bound
args.lbx(2:num_con:(n_states*(N_MHE+1) + n_disturbances*N_MHE),1) = Q_min;  % Generated Heat Lower Bound
args.ubx(2:num_con:(n_states*(N_MHE+1) + n_disturbances*N_MHE),1) = Q_max;  % Generated Heat Upper Bound

% Disturbances constraints
args.lbx(3:num_con:((n_states+n_disturbances)*N_MHE),1) = -inf;             % Noise Surface T Lower Bound
args.ubx(3:num_con:((n_states+n_disturbances)*N_MHE),1) = inf;              % Noise Surface T Upper Bound
args.lbx(4:num_con:((n_states+n_disturbances)*N_MHE),1) = -inf;             % Noise Generated Heat Lower Bound
args.ubx(4:num_con:((n_states+n_disturbances)*N_MHE),1) = inf;              % Noise Generated Heat Upper Bound

%% Moving Horizon Estimation
%% Initial Guess
x1_initial_guess = R_Temper1(1);
x2_initial_guess = 0;


x_ig = [x1_initial_guess ; x2_initial_guess];
X0(1,1:n_states) = x_ig';

X_estimate = [];                    % X_estimate contains the MHE estimate of the states
X0 = x_ig'.*ones(N_MHE+1,n_states); % initialization of the states decision variables

W_estimate = [];
W0 = rand(N_MHE,n_disturbances);

%% Start Moving Horizon Estimation

mheiter = 0;

y_measurements = R_Temper1;
u_cl = awgn(R_Tamb1,10);% R_Tamb1;

P_cov = diag([1 1]).^2;

data_length = 3601*1;

for k = 1: length(t11(1:data_length)) - (N_MHE)

    mheiter = k

    % Get the Measurement Window and Put It as Parameters in P
    args.p   = [y_measurements(k:k+N_MHE,1)',u_cl(k:k+N_MHE,:)',reshape(P_cov,1,n_states*n_states),X0(1,:)];

    % initial value of the optimization variables
    args.x0  = [reshape(X0',n_states*(N_MHE+1),1) ; reshape(W0',n_disturbances*N_MHE,1)];

    % Solve Optimization Problem
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    
    % Get Solution TRAJECTORY
    X_sol = reshape(full(sol.x(1:n_states*(N_MHE+1)))',n_states,N_MHE+1)'; 
    W_sol = reshape(full(sol.x(n_states*(N_MHE+1)+1:end))',n_disturbances,N_MHE)';
    
    % Save the TRAJECTORY
    X_estimate = [X_estimate;X_sol(N_MHE+1,:)];
    W_estimate = [W_estimate;W_sol(N_MHE,:)];
    
    % Shift trajectory to initialize the next step
    X0 = [X_sol(2:end,:);X_sol(end,:)];
    W0 = [W_sol(2:end,:);W_sol(end,:)];

    % Algebraic Riccati Equation
    P_cov = B_discrete.*State_cov.*B_discrete' + A_discrete*(P_cov - P_cov*C'*inv(Meas_cov + C_discrete*P_cov*C_discrete')*C_discrete*P_cov)*A_discrete';
    
end

X_estimate = [x_ig'.*ones(N_MHE,n_states) ; X_estimate];

%% Figure

figure
subplot(311)
plot(t11(1:data_length)/3600,R_Current1(1:data_length))
ylabel('Current')

subplot(312)
plot(t11(1:data_length)/3600,R_Temper1(1:data_length))
hold on
plot(t11(1:data_length)/3600,X_estimate(:,1))
ylabel('Temperature')

subplot(313)
plot(t11(1:data_length)/3600,X_estimate(:,2))
hold on;
plot(t11(1:data_length)/3600, R_Res(1:data_length).*R_Current1(1:data_length).^2)
ylabel('Heat Generation')
set(gcf,'Color','White')