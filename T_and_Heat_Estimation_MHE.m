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
addpath('/Users/moonjihoon/Library/CloudStorage/OneDrive-ThePennsylvaniaStateUniversity/Research/Code/Battery T and Heat Estimation using MHE/casadi-3')

import casadi.*

%% Time Setting
dt    = 0.1;        % Sampling Time [s]
N_MHE = 10;         % Horizon Length

%% State limits
Charge_current = 4.85;
R0_1st_cycle   = 0.061;

Q_min = 0;                                              % Minimum Generated Heat
Q_max = 2*R0_1st_cycle*Charge_current^2;                % Maximum Generated Heat

T_min = 10;                                             % Minimum Internal Temperature
T_max = 50;                                             % Maximum Internal Temperature

Tamb_min = 20;                                          % Minimum Ambient Temperature
Tamb_max = 30;                                          % Maximum Ambient Temperature

%% States
Ti = SX.sym('Ti');                                      % State 1: Internal Temperature
Q_h = SX.sym('Q_h');                                    % State 2: Generated Heat

states = [Ti; Q_h];                                     % State Vector
n_states = length(states);                              % Number of States

%% Control
T_amb = SX.sym('T_amb');                                % Control Input: Ambient Temperature

controls = [T_amb];                                     % Control Input
n_controls = length(controls);                          % Number of Control Input

%% Disturbances
w1 = SX.sym('w1');                                      % State 1 Process Noise
w2 = SX.sym('w2');                                      % State 2 Process Noise

disturbances   = [w1 ; w2];                             % Disturbances Vector
n_disturbances = length(disturbances);                  % Number of Disturbances

%% Covariance Matrix
Meas_noise_cov = 1;                                     % Measuremet Noise Covariance Matrix Value
Meas_cov=(diag(Meas_noise_cov)).^2;                     % Measuremet Noise Covariance Matrix

State_noise_cov=10;                                     % Process Noise Covariance Matrix Value
State_cov=diag([State_noise_cov,State_noise_cov]).^2;   % Process Noise Covariance Matrix

%% State Dynamics

% Define State Space Model
A = [0.9999, 0.0006749 ; 0, 0];
B = [0.0001289 ; 0];
Cd = [1, 0];
Dd = 0;

rhs = A*[Ti; Q_h] + B*T_amb;                            % System Right Hand Side
f = Function('f',{states,controls},{rhs});              % State Dynamics Function

measurement_rhs = Ti;
h = Function('h',{states,controls},{measurement_rhs});  % Measurement Function

%% Decision Variables
X=SX.sym('X',n_states, (N_MHE+1));                      % Decision Variables for State
Wj=SX.sym('Wj',n_disturbances, (N_MHE));                % Decision Variables for Process Noise

%% Parameters to be Passed: Measuremnts and control inputs
P = SX.sym('P',1, (N_MHE+1)+ (N_MHE+1)); % first N_MHE+a correspond to surface temperature measurments and second N_MHE columns Correspond to Control inputs which are known

%% Weighting Matrices
V = inv(sqrt(Meas_cov));          % Weighting Matrix for Measurement
W = inv(sqrt(State_cov));         % Weighting Matrix for State

%% Define Ojective Function
obj = 0; % Objective function
g = [];  % constraints vector

% Output Surface Temperature
for k = 1:N_MHE+1

    st = X(:,k);
    con = P(:,(N_MHE+1+k));
    h_x = h(st,con);
    y_tilde = P(:,k);
    
    obj = obj+ (y_tilde-h_x)' * V * (y_tilde-h_x); % calculate obj based on difference between measured temperature and measurment model

end

% State or Process Noise
for k = 1:N_MHE

    st_pr = X(:,k);
    st_po= X(:,k+1);
    con=P(:, (N_MHE+1+k));
    dist=Wj(:,k);
    
    obj = obj+ (st_po-(f(st_pr,con)+dist))' * W * (st_po-(f(st_pr,con)+dist)); 

end

% Multiple Shooting Constraints
for k = 1:N_MHE
    
    st = X(:,k);  
    con = P(:,(N_MHE+1+k));
    dist = Wj(:,k);
    st_next = X(:,k+1);
    f_value = f(st,con,dist);
    st_next_euler = st+ (T*f_value);

    g = [g;st_next-st_next_euler]; % compute constraints

end

%% MHE solver
% Make the Decision Variable One Column  Vector
OPT_variables = [reshape(X,n_states*(N_MHE+1),1) ; reshape(N,n_disturbances*(N_MHE),1)];

% Structure
nlp_mhe = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

% Option
opts = struct;
opts.ipopt.max_iter = 4000;
opts.ipopt.print_level = 0;
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-10;
opts.ipopt.acceptable_obj_change_tol = 1e-8;

% Solver
solver = nlpsol('solver', 'ipopt', nlp_mhe,opts);

args = struct;

%% Constraint 

num_con = n_states + n_disturbances;

% equality constraints
args.lbg(1:n_states*(N_MHE)) = 0;
args.ubg(1:n_states*(N_MHE)) = 0;

% State constraints
args.lbx(1:num_con:(n_states*(N_MHE+1) + n_disturbances*N_MHE),1) = 0; %state x1 lower bound
args.ubx(1:num_con:(n_states*(N_MHE+1) + n_disturbances*N_MHE),1) = 0.00001; %state x1 upper bound
args.lbx(2:num_con:(n_states*(N_MHE+1) + n_disturbances*N_MHE),1) = 0; %state x2 lower bound
args.ubx(2:num_con:(n_states*(N_MHE+1) + n_disturbances*N_MHE),1) = 5; %state x2 upper bound

% Disturbances constraints
args.lbx(3:num_con:((n_states+n_disturbances)*N_MHE),1) = -inf; % noise x1 lower bound
args.ubx(3:num_con:((n_states+n_disturbances)*N_MHE),1) = inf; % noise x1 upper bound
args.lbx(4:num_con:((n_states+n_disturbances)*N_MHE),1) = -inf; % noise x2 lower bound
args.ubx(4:num_con:((n_states+n_disturbances)*N_MHE),1) = inf; % noise x2 upper bound
