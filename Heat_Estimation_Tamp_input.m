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
addpath('C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Documents\Battery-Temperature-Estimation-using-Moving-Horizon-Estimation-MHE\casadi-3')

% Mac
% addpath('/Users/moonjihoon/Library/CloudStorage/OneDrive-ThePennsylvaniaStateUniversity/Research/Code/Batttery T and Heat Estimation using MHE/casadi-3')

import casadi.*

%% Input Profile
folder_path='C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Documents\MHE_Temperature_Estimation\14226830';
[fileName, folderPath] = uigetfile('*.mat', 'Select the mat file', folder_path);
load(fullfile(folderPath,fileName));
charge_start=[batteryData.chargePhases.Charge_start];
start_index=charge_start(1);
% last_index=length(batteryData.Time_s);
time1=batteryData.Time_s;
time1=time1-time1(1);
curr1=batteryData.Current;
volt1=batteryData.Voltage_V;
Temp1=batteryData.Temperature_C;
Tamb=batteryData.T_amb;
Cycle_no1=batteryData.cycle_no;
Resistance=batteryData.Resistance;
Resistance(1:start_index)=Resistance(start_index+1);
dt=diff(time1(1:2));
Charge_current = -3;
R0_1st_cycle   = Resistance(1);
Q_min          = 0;                                                         % Minimum Generated Heat
Q_max          = 100;                           % Maximum Generated Heat
T_min          = 20;                                                        % Minimum Surface Temperature
T_max          = 100;                                                        % Maximum Surface Temperature
Tamb_min       = 15;                                                        % Minimum Ambient Temperature
Tamb_max       = 30;
theta=batteryData.theta;
%% Set Horizon Length
N_MHE = 60;     
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

states   = [Ti; Q_h];                                    % State Vector
n_states = length(states);                                      % Number of States

%% Control
T_amb      = SX.sym('T_amb');                                   % Control Input: Ambient Temperature

controls   = [T_amb];                                           % Control Input
n_controls = length(controls);                                  % Number of Control Input

%% Covariance Matrix
Meas_noise_cov  = 1;                                            % Measuremet Noise Covariance Matrix Value
Meas_cov        = diag(Meas_noise_cov).^2;                      % Measuremet Noise Covariance Matrix

State_noise_cov = 1000;                                         % Process Noise Covariance Matrix Value
State_cov       = diag([State_noise_cov,State_noise_cov]).^2;   % Process Noise Covariance Matrix

%% State Dynamics
rhs = A_continuous*[Ti; Q_h] + B_continuous*T_amb;              % System Right Hand Side
f   = Function('f',{states,controls},{rhs});       % State Dynamics Function

measurement_rhs = Ti;
h = Function('h',{states,controls},{measurement_rhs});          % Measurement Function

%% Decision Variables
X  = SX.sym('X',n_states, (N_MHE+1));                           % Decision Variables for State

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
    f_value = f(st_pr,con);
  
    obj = obj + (st_po - (st_pr + dt*f_value))' * W * (st_po - (st_pr + dt*f_value)); % calculate obj

end

% Multiple Shooting Constraints
for k = 1:N_MHE
    
    st            = X(:,k);  
    con           = P(:,(N_MHE+1+k));
    st_next       = X(:,k+1);
    f_value       = f(st,con);
    st_next_euler = st + (dt*f_value);

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
args.lbg(1:n_states*(N_MHE)) = 0;
args.ubg(1:n_states*(N_MHE)) = 0;

% State constraints
args.lbx(1:num_con:(n_states*(N_MHE+1)),1) = T_min;  % Surface T Lower Bound
args.ubx(1:num_con:(n_states*(N_MHE+1)),1) = T_max;  % Surface T Upper Bound
args.lbx(2:num_con:(n_states*(N_MHE+1)),1) = Q_min;  % Generated Heat Lower Bound
args.ubx(2:num_con:(n_states*(N_MHE+1)),1) = Q_max;  % Generated Heat Upper Bound

%% Moving Horizon Estimation
%% Initial Guess
x1_initial_guess = Temp1(1);
x2_initial_guess = 0;


x_ig = [x1_initial_guess ; x2_initial_guess];
X0(1,1:n_states) = x_ig';

X_estimate = [];                    % X_estimate contains the MHE estimate of the states
X0 = x_ig'.*ones(N_MHE+1,n_states); % initialization of the states decision variables

%% Start Moving Horizon Estimation

mheiter = 0;

y_measurements = Temp1;
u_cl=Tamb;

P_cov = diag([10 1]).^2;

data_length = length(time1);

for k = 1: length(time1(1:data_length)) - (N_MHE)

    mheiter = k

    % Get the Measurement Window and Put It as Parameters in P
    args.p   = [y_measurements(k:k+N_MHE,1)',u_cl(k:k+N_MHE,:)',reshape(P_cov,1,n_states*n_states),X0(1,:)];

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
    
end

X_estimate = [x_ig'.*ones(N_MHE,n_states) ; X_estimate];
X11=X_estimate;
X11=X11(N_MHE+1:end,:);
X22=X11(N_MHE+1:end,:);
%% Figure
data_length=length(X11);
figure
ax1=subplot(311)
plot(time1(1:data_length),curr1(1:data_length))
ylabel('Current')

ax2=subplot(312)
plot(time1(1:data_length),Temp1(1:data_length))
hold on
plot(time1(1:data_length),X11(1:data_length,1),'--k')
ylabel('Temperature')


ax3=subplot(313)

hold on;
plot(time1(1:data_length), Resistance(1:data_length).*curr1(1:data_length).^2)
plot(time1(1:data_length),X11(1:data_length,2),'--k')
ylabel('Heat Generation')
set(gcf,'Color','White')
linkaxes([ax1,ax2,ax3],'x')
%% Resistance Calculation
[~, Ambient_Temp, Known_Res] = extractChargePhaseData(batteryData);
charge_start=[batteryData.chargePhases.Charge_start];
%% Plotting Setup
figure;
Charge_Curr=3;
for j = 1:length(charge_start)
    % Determine index range for current charge phase
    time_charge=batteryData.chargePhases(j).Time_s;
    Current_charge=batteryData.chargePhases(j).Current_A;
    cs=batteryData.chargePhases(j).Charge_start;

    % Find end of charging period
    index_end = findChargingEnd(Current_charge,Charge_Curr);
    Heat=  X11(cs+1:cs+index_end, 2);
    Known_Res(j)=batteryData.chargePhases(j).Resistance;
    Heat_orig=Known_Res(j).*Current_charge.^2;
    TS_MHE_index=find(time_charge>=300,1);
    plot(time_charge(TS_MHE_index:index_end),Heat(TS_MHE_index:end));
    hold on;
    plot(time_charge(TS_MHE_index:index_end),Heat_orig(TS_MHE_index:index_end),'--k');
    R_estimated(j)=mean(Heat(TS_MHE_index:end))/Charge_Curr^2;
    clear Heat
    clear time_charge Heat_orig

end
xlabel('time of charge')
ylabel('Heat Generation')
batteryData.Estimated_Resistance_MHE=R_estimated;


%%  Create the plot
figure;
hold on;
plot([batteryData.chargePhases.cycle_no], [batteryData.chargePhases.Resistance], '.k', 'MarkerSize', 10)
plot([batteryData.chargePhases.cycle_no], R_estimated, '.r', 'MarkerSize', 10)
legend('True Value', 'MHE based' )
xlabel('Cycle no');
ylabel('Resistance');
grid on;
ylim([0.02,0.05])
% Customize the plot appearance
set(gca, 'FontSize', 12);
set(gcf, 'Position', [100, 100, 800, 600]);

RMSE=sqrt(mean(([batteryData.chargePhases.Resistance]-R_estimated).^2));
title([fileName, ' MHE RMSE: ', num2str(RMSE)]);
targetLocation = fullfile('C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Documents\Battery-Temperature-Estimation-using-Moving-Horizon-Estimation-MHE\Results', [fileName, 'MHE.png']);  % You can change the extension to match the desired format

% Save the figure
saveas(gcf, targetLocation);
%%
function [charge_start, Ambient_Temp, Known_Res] = extractChargePhaseData(batteryData)
charge_start = [];
Ambient_Temp = [];
Known_Res = [];
for j = 1:length(batteryData.chargePhases)
    if ~isempty(batteryData.chargePhases(j).Charge_start)
        charge_start = [charge_start, batteryData.chargePhases(j).Charge_start];
        Ambient_Temp = [Ambient_Temp, batteryData.chargePhases(j).Tamb_C];
        Known_Res = [Known_Res, batteryData.chargePhases(j).Resistance];
    end
end
end

function index_end = findChargingEnd(Current_A,Charge_Curr)
positive_indices = find(Current_A< -0.99*Charge_Curr);
if any(diff(positive_indices)>1)
    positive_indices=positive_indices(1:find(diff(positive_indices)>1));
end
if ~isempty(positive_indices)
    index_end =  positive_indices(end) - 1;
else
    warning('No positive current found in this charge phase.');
    index_end=1;
end
end





