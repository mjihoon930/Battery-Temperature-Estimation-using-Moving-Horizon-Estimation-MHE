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