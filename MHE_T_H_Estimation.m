%%  Project: Temperature and Generated Heat Estimation using Moving Horizon Estimation (MHE)
%   Date: 03/14/2024
%
%   Member: Nitisha Ahuja, Jihoon Moon
% 
%   Pennsylvania State University
%   Mechanical Engineering
%   Mechatronics Research Laboratory
%   https://www.me.psu.edu/mrl/

clear all
close all
clc

%% Import CasADi Package
addpath('C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Desktop\casadi-3.6.4-windows64-matlab2018b')
import casadi.*

%% Input Profile
folder_path='C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Documents\Battery-Temperature-Estimation-using-Moving-Horizon-Estimation-MHE\Data\14226830';
[fileName, folderPath] = uigetfile('*.mat', 'Select the mat file', folder_path);
load(fullfile(folderPath,fileName));
for i=1:length(batteryData.chargePhases)
start_index=batteryData.chargePhases(i).Charge_start;
if ~isempty(start_index)
    break;
end
end
last_index=length(batteryData.Time_s);
time1=batteryData.Time_s(start_index:last_index);
curr1=-batteryData.Current(start_index:last_index);
volt1=batteryData.Voltage_V(start_index:last_index);
Temp1=batteryData.Temperature_C(start_index:last_index);
Tamb=batteryData.T_amb(start_index:last_index);
Cycle_no1=batteryData.cycle_no(start_index:last_index);
Resistance=batteryData.Resistance(start_index:last_index);
dt=diff(time1(1:2));
Charge_current = -3;
R0_1st_cycle   = Resistance(1);
Q_min          = 0;                                                         % Minimum Generated Heat
Q_max          = 2*R0_1st_cycle*Charge_current^2;                           % Maximum Generated Heat
T_min          = 20;                                                        % Minimum Surface Temperature
T_max          = 40;                                                        % Maximum Surface Temperature
Tamb_min       = 20;                                                        % Minimum Ambient Temperature
Tamb_max       = 25;
theta=batteryData.theta;
%% Set Horizon Length
N_MHE = 20;     
%% System Dynamics
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
T_amb      = SX.sym('T_amb');
states   = [Ti; Q_h; T_amb];                                    % State Vector
n_states = length(states);                                      % Number of States
%% Covariance Matrix
Meas_noise_cov  = 100;                                            % Measuremet Noise Covariance Matrix Value
Meas_cov        = diag(Meas_noise_cov).^2;                      % Measuremet Noise Covariance Matrix                                    % Process Noise Covariance Matrix Value
State_cov       = diag([1,10]).^2;                              % Process Noise Covariance Matrix
P_cov = diag([0.1 10]).^2;
%% State Dynamics
rhs = A_continuous*[Ti; Q_h] + B_continuous*T_amb;              % System Right Hand Side
f   = Function('f',{states},{rhs});       % State Dynamics Function
measurement_rhs = Ti;
h = Function('h',{states},{measurement_rhs});          % Measurement Function

%% Decision Variables
X  = SX.sym('X',n_states, (N_MHE+1));                           % Decision Variables for State

%% Parameters to be Passed: Measuremnts and control inputs
P  = SX.sym('P',1, (N_MHE+1) + (n_states-1)*(n_states-1) + (n_states-1));                       % first N_MHE+a correspond to surface temperature measurments and second N_MHE columns Correspond to Control inputs which are known

%% Weighting Matrices
V = inv(sqrt(Meas_cov));                                        % Weighting Matrix for Measurement
W = inv(sqrt(State_cov));                                       % Weighting Matrix for State

%% Define Ojective Function

obj = 0; % Objective function
g   = [];  % constraints vector

% Arrival Cost
first_state = X(1:2,1);
first_state_prior = reshape(P(:,(end-(n_states-1)+1):end),(n_states-1),1);

P_arrival = inv(reshape(P(:,((N_MHE+1)+1):((N_MHE+1))+((n_states-1)*(n_states-1))),(n_states-1),(n_states-1)));

obj = obj + ( (first_state - first_state_prior)' * P_arrival * (first_state - first_state_prior) );

% Output Surface Temperature
for k = 1:N_MHE+1

    st      = X(:,k);
    h_x     = h(st);
    y_tilde = P(:,k);
    
    obj     = obj + (y_tilde - h_x)' * V * (y_tilde - h_x);     % calculate obj based on difference between measured temperature and measurment model

end

% State or Process Noise
for k = 1:N_MHE

    st_pr  = X(:,k);
    st_po  = X(1:2,k+1);
    
    obj = obj + ((st_po - (A_discrete*st_pr(1:2) + B_discrete*st_pr(3)))' * W * (st_po - (A_discrete*st_pr(1:2) + B_discrete*st_pr(3))));

end

% Multiple Shooting Constraints
for k = 1:N_MHE
    
    st            = X(:,k);  
    st_next       = X(1:2,k+1);
    f_value       = f(st);
    st_next_euler = st(1:2) + (dt*f_value);

    g = [g;st_next-st_next_euler]; % compute constraints

end

%% MHE solver
% Make the Decision Variable One Column  Vector
OPT_variables = [reshape(X,n_states*(N_MHE+1),1)];

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

num_con = n_states;

% equality constraints
args.lbg(1:(n_states-1)*(N_MHE)) = 0;
args.ubg(1:(n_states-1)*(N_MHE)) = 0;

% State constraints
args.lbx(1:num_con:(n_states*(N_MHE+1)),1) = T_min;  % Surface T Lower Bound
args.ubx(1:num_con:(n_states*(N_MHE+1)),1) = T_max;  % Surface T Upper Bound
args.lbx(2:num_con:(n_states*(N_MHE+1)),1) = Q_min;  % Generated Heat Lower Bound
args.ubx(2:num_con:(n_states*(N_MHE+1)),1) = Q_max;  % Generated Heat Upper Bound
args.lbx(3:num_con:(n_states*(N_MHE+1)),1) = Tamb_min;  % Generated Heat Lower Bound
args.ubx(3:num_con:(n_states*(N_MHE+1)),1) = Tamb_max;  % Generated Heat Upper Bound

%% Moving Horizon Estimation
%% Initial Guess
x1_initial_guess = Temp1(1);
x2_initial_guess = 0;
x3_initial_guess = 23;

x_ig = [x1_initial_guess ; x2_initial_guess ; x3_initial_guess];
X0(1,1:n_states) = x_ig';

X_estimate = [];                    % X_estimate contains the MHE estimate of the states
X0 = x_ig'.*ones(N_MHE+1,n_states); % initialization of the states decision variables

%% Start Moving Horizon Estimation

mheiter = 0;

y_measurements = Temp1;


data_length = 20000;%length(time1);

for k = 1: length(time1(1:data_length)) - (N_MHE)

    mheiter = k

    % Get the Measurement Window and Put It as Parameters in P
    args.p   = [y_measurements(k:k+N_MHE,1)',reshape(P_cov,1,(n_states-1)*(n_states-1)),X0(1,1:2)];

    % initial value of the optimization variables
    args.x0  = [reshape(X0',n_states*(N_MHE+1),1)];

    % Solve Optimization Problem
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    
    % Get Solution TRAJECTORY
    X_sol = reshape(full(sol.x(1:n_states*(N_MHE+1)))',n_states,N_MHE+1)'; 
    
    % Save the TRAJECTORY
    X_estimate = [X_estimate;X_sol(N_MHE+1,:)];
    
    % Shift trajectory to initialize the next step
    X0 = [X_sol(2:end,:);X_sol(end,:)];

    % Algebraic Riccati Equation
    P_cov = B_discrete.*State_cov.*B_discrete' + A_discrete*(P_cov - P_cov*C'*inv(Meas_cov + C_discrete*P_cov*C_discrete')*C_discrete*P_cov)*A_discrete';
    if batteryData.Resistance(k)>0
        HG(k)=X_estimate(k,2);
        Res_HG(k)=X_estimate(k,2)./curr1(k).^2;
    else 
        HG(k)=0;
        Res_HG(k)=0;
    end
end

X_estimate = [x_ig'.*ones(N_MHE,n_states) ; X_estimate];

%% Figure

figure
subplot(311)
plot(time1(1:data_length)/3600,curr1(1:data_length))
ylabel('Current')

subplot(312)
plot(time1(1:data_length)/3600,Temp1(1:data_length))
hold on
plot(time1(1:data_length)/3600,X_estimate(:,1))
ylabel('Temperature')

subplot(313)
plot(time1(1:data_length)/3600,X_estimate(:,2))
hold on;
plot(time1(1:data_length)/3600, Resistance(1:data_length).*curr1(1:data_length).^2)
ylabel('Heat Generation')
set(gcf,'Color','White')

figure;
plot(Res_HG);
hold on;
plot(Resistance);
