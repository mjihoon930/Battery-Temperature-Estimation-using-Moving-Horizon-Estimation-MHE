% startup.m

% Add Matlab_tools directory and all its subfolders to the MATLAB path
addpath(genpath('C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Documents\Matlab_tools'));

% Add the tbxmanager directory to the MATLAB path
addpath('C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Desktop\Model_Predictive_Control\tbxmanager');

% Optionally, add all subfolders in tbxmanager as well
addpath(genpath('C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Desktop\Model_Predictive_Control\tbxmanager'));
addpath('C:\gurobi1003\win64\matlab');
% Display a message indicating that paths have been set

addpath(genpath('C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Documents\Matlab_tools_data'));
disp('MATLAB paths have been set, including all subfolders in Matlab_tools and tbxmanager.');
startup;
gurobi_setup;