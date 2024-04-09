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
addpath('/Users/moonjihoon/Library/CloudStorage/OneDrive-ThePennsylvaniaStateUniversity/Research/Code/Code/Rsc Estimation with MHE/casadi-3')
 import casadi.*

dt = 0.1; %[s]
N_MHE = 10; % Estimation horizon
%
Charge_current=4.85;
R0_1st_cycle=0.061;

%% State limits
Q_max = 2*R0_1st_cycle*Charge_current^2; Q_min  = 0;
T_min = 10; T_max=50;
Tamb_min=20;
Tamb_max=30;

%% States
Ti = SX.sym('Ti'); Q_h = SX.sym('Q_h'); %% two states of the system 
states = [Ti; Q_h]; n_states = length(states);
%% Control
T_amb = SX.sym('T_amb');
controls = [T_amb]; n_controls = length(controls);
%% Disturbances
w1 = SX.sym('w1');
w2 = SX.sym('w2');
disturbances = [w1 ; w2];
n_disturbances = length(disturbances);
%% Covariance Matrix
Meas_noise_cov=1;
Meas_cov=(diag(Meas_noise_cov)).^2;
State_noise_cov=10;
State_cov=diag([State_noise_cov,State_noise_cov]).^2;

%% State Dynamics
Ad=[0.9999, 0.0006749;0,1];
Bd=[0.0001289;0];
Cd=[1,0];
Dd=0;

rhs = Ad*[Ti; Q_h]+Bd*T_amb; % system r.h.s
f = Function('f',{states,controls},{rhs}); % State dynamics

measurement_rhs = Ti;
h = Function('h',{states,controls},{measurement_rhs}); % MEASUREMENT MODEL

%% Decision variables
X=SX.sym('X',n_states, (N_MHE+1));
% State Noise wj
Wj=SX.sym('Wj',n_disturbances, (N_MHE));

%% Parametrs to be passed: Measuremnts and control inputs
P = SX.sym('P',1, (N_MHE+1)+ (N_MHE+1)); % first N_MHE+a cporrespond to surface temperature measurments and second N_MHE columns cprresopo
% to Control inputs which are known

V = inv(sqrt(Meas_cov));        % weighing matrices (output)  y_tilde - y
W = inv(sqrt(State_cov));         % weighing matrices (input)   u_tilde - u

%%

obj = 0; % Objective function
g = []; % constraints vector

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
obj = obj+ (st_po-(f(st_pr,con)+dist))' * W * (st_po-(f(st_pr,con)+dist)); % calculate obj
end
