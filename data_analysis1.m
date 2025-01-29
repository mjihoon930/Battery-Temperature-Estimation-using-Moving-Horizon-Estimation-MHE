%% This script is created for processing the data from the battery pack or cell to
%% get .mat format of V, I, T, along with cycling number and ambient temperature
clear all
close all
clc
%% Load Test File
folder_path='C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Documents\MHE_Temperature_Estimation\14226830';
% Prompt the user to select a CSV file
[fileName, folderPath] = uigetfile('*.csv', 'Select a csv file', folder_path);

% Check if the user selected a file or canceled the operation
if isequal(fileName, 0)
    disp('User canceled the file selection.');
    return;
else
    % Construct the full file path
    fullFilePath = fullfile(folderPath, fileName);
    
    % Load the CSV file into a table
    dataTable = readtable(fullFilePath);
end

%% Modify the following code based on the data header label

desiredSamplingRate = 1/5; % 1 Hz, for example. Adjust as needed.
[time_s, Current_A, Voltage_V, Temperature_C] = resampleData(dataTable, desiredSamplingRate);

% Create a structure to hold battery data
batteryData.Current = Current_A;
batteryData.Voltage_V = Voltage_V;
batteryData.Time_s = time_s;
batteryData.Temperature_C = Temperature_C;
batteryData.T_amb = zeros(length(Temperature_C),1);
batteryData.cycle_no=zeros(length(Temperature_C),1);
batteryData.Resistance=zeros(length(Temperature_C),1);

fd;
plot(batteryData.Time_s,batteryData.Current );
fd;
plot(batteryData.Time_s,batteryData.Voltage_V );
fd;
plot(batteryData.Time_s,batteryData.Temperature_C);
Crate=3;
%% Initialize variables
chargePhases = struct();  % Structure to store all charge phases
isCharging = false;
phaseIndex = 1;
R0=0;
Res = [];
deltaI=0;
T_amb=0;
% Loop through the data to identify and separate charge phases
for i = 1:height(batteryData.Current)
    if batteryData.Current(i) < 0  % Negative current indicates charging
        CC_startIndex = i;
        % Calculate resistance if conditions are met
        if ((batteryData.Current(i)) < -1.6)
            % Find rest index (last point before significant negative increase)
            restIndex = max(1, i - 1);
            while restIndex > 1 && batteryData.Current(restIndex) < -0.1
                restIndex = restIndex - 1;
            end
            Change_I_var = abs(batteryData.Current(CC_startIndex) - batteryData.Current(restIndex));
            if CC_startIndex < height(batteryData.Current) && Change_I_var > Crate * 0.95 && ~isCharging
                Change_I_var = abs(batteryData.Current(CC_startIndex) - batteryData.Current(restIndex));
                deltaV = batteryData.Voltage_V(CC_startIndex) - batteryData.Voltage_V(restIndex);
                deltaI = batteryData.Current(CC_startIndex) - batteryData.Current(restIndex);
            end
            if deltaI ~= 0 && ~isCharging
                R0 = abs(deltaV / deltaI);
                chargePhases(phaseIndex).Charge_start = i;
            Amb_T_index = min(i + 1, height(batteryData.Current));
            % Storing the last temperature value before discharging start is stored as ambient temperature
            while Amb_T_index < height(batteryData.Current) && batteryData.Current(Amb_T_index) <= 0
                Amb_T_index = Amb_T_index + 1;
            end
            T_amb = batteryData.Temperature_C(Amb_T_index-1);
            chargePhases(phaseIndex).Time_s = batteryData.Time_s(restIndex: Amb_T_index-1)-batteryData.Time_s(restIndex);
            chargePhases(phaseIndex).Current_A = batteryData.Current(restIndex: Amb_T_index-1);
            chargePhases(phaseIndex).Voltage_V =batteryData.Voltage_V(restIndex: Amb_T_index-1);
            chargePhases(phaseIndex).Temperature_C = batteryData.Temperature_C(restIndex: Amb_T_index-1);
            chargePhases(phaseIndex).Tamb_C =T_amb ;
            chargePhases(phaseIndex).Resistance = R0;
            chargePhases(phaseIndex).cycle_no =phaseIndex;
            phaseIndex = phaseIndex + 1;
            currentPhase = [];  % Reset current phase
            isCharging=1;
        deltaI=0;
        end
        end
    else
        isCharging=0;
    end
    batteryData.T_amb(i)=T_amb;
    batteryData.Resistance(i)=R0;
end
batteryData.T_amb(batteryData.T_amb==0)=mean(batteryData.T_amb);
outlier_indices = isoutlier([chargePhases.Resistance] , 'movmedian',100);
filtered_charge_phases = chargePhases(~outlier_indices);
%% Optional: Plot temperature growth for each charge phase
% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');
% Number of phases
numPhases = length(filtered_charge_phases );

% Create a continuous colormap
cmap = jet(numPhases);
colormap(cmap);
k=0;

for j = 1:numPhases
    Voltage_V = filtered_charge_phases (j).Voltage_V;
    %     if max(abs(chargePhases(j).Current_A)) > 1.7 && Voltage_V(1) > 3.45
    plot(filtered_charge_phases(j).Time_s./60, filtered_charge_phases (j).Temperature_C-filtered_charge_phases (j).Temperature_C(1), 'Color', cmap(j,:), 'LineWidth', 0.5);
    hold on;
    k=k+1;
    cycle_no(k)=filtered_charge_phases(j).cycle_no;
end

title('Temperature Evolution over Charge Phases');
xlabel('Time [min]');
ylabel('Temperature [C]');

set(axes1,'CLim',[1 847]);
% Create colorbar
colorbar(axes1);
% Adjust figure size for better visibility
set(gcf, 'Position', [100, 100, 1200, 800]);

hold off;

%% Optional: Plot temperature growth for each charge phase
% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');
% Number of phases
numPhases = length(filtered_charge_phases );

% Create a continuous colormap
cmap = jet(numPhases);
colormap(cmap);
k=0;

for j = 1:numPhases
    Voltage_V = filtered_charge_phases (j).Voltage_V;
    %     if max(abs(chargePhases(j).Current_A)) > 1.7 && Voltage_V(1) > 3.45
    plot(filtered_charge_phases(j).Time_s./60, filtered_charge_phases (j).Voltage_V, 'Color', cmap(j,:), 'LineWidth', 0.5);
    hold on;
    k=k+1;
    cycle_no(k)=filtered_charge_phases(j).cycle_no;
end

title('Violtage Evolution over Charge Phases');
xlabel('Time [min]');
ylabel('Temperature [C]');

set(axes1,'CLim',[1 847]);
% Create colorbar
colorbar(axes1);
% Adjust figure size for better visibility
set(gcf, 'Position', [100, 100, 1200, 800]);

hold off;
% %% Calling Model
% nc=2; %no_of_cycles_training
% s_i=filtered_charge_phases(10).Charge_start;
% e_i=filtered_charge_phases(nc+10).Charge_start;
% Thermal_model_batteryData.Current_A=batteryData.Current(s_i:e_i);
% Thermal_model_batteryData.Time_s=batteryData.Time_s(s_i:e_i);
% Thermal_model_batteryData.Temperature_C=batteryData.Temperature_C(s_i:e_i);
% Thermal_model_batteryData.Resistance=batteryData.Resistance(s_i:e_i);
% Thermal_model_batteryData.T_amb=batteryData.T_amb(s_i:e_i);

%[theta, RMSE, MAE, Sys] =Thermal_Model_with_Gradient_Descent(Thermal_model_batteryData);
%
% [theta, RMSE, MAE, Sys] =Thermal_Model_time_Series_Data(Thermal_model_batteryData)
nc=5;
for i=1
[theta, RMSE, MAE] = Thermal_Model(filtered_charge_phases(i:i+nc));
store_alpha(i)=theta(1);
store_beta(i)=theta(2);
close all;
end
batteryData.theta=theta;

batteryData.chargePhases=filtered_charge_phases;
%%
folder_path='C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Documents\MHE_Temperature_Estimation\14226830';
% Prompt the user to select a CSV file
[fileName, folderPath] = uigetfile('*.csv', 'Select Impedance file', folder_path);

% Check if the user selected a file or canceled the operation
if isequal(fileName, 0)
    disp('User canceled the file selection.');
    return;
else
    % Construct the full file path
    fullFilePath = fullfile(folderPath, fileName);
    
    % Load the CSV file into a table
    dataTable1 = readtable(fullFilePath);
end
%% Calling Luenberger Observer
Charge_Curr=3;
theta(1)=store_alpha(1);
theta(2)=store_beta(1);
[R_estimated] = Luenberger_Observer(theta, batteryData, Charge_Curr);
% [Heat, R_estimated, Known_Res] = Luenberger_Observer1(store_alpha,store_beta, batteryData, Charge_Curr);
% [Heat , R_estimated,store_alpha1, store_beta1]=Luenberger_Observer_Adaptive(theta,batteryData, Charge_Curr)
batteryData.Estimated_Resistance=R_estimated;

%%  Create the plot
figure;
hold on;
plot([filtered_charge_phases.cycle_no], [batteryData.chargePhases.Resistance], '.k', 'MarkerSize', 10)
plot([filtered_charge_phases.cycle_no], R_estimated, '.r', 'MarkerSize', 10)
legend('True Value', 'Observer based' )
xlabel('Cycle no');
ylabel('Resistance');
grid on;

% Customize the plot appearance
set(gca, 'FontSize', 12);
set(gcf, 'Position', [100, 100, 800, 600]);

RMSE=sqrt(mean((Known_Res-R_estimated).^2));
title([fileName, ' RMSE: ', num2str(RMSE)]);
targetLocation = fullfile('C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Documents\Battery-Temperature-Estimation-using-Moving-Horizon-Estimation-MHE\Results', [fileName, '.png']);  % You can change the extension to match the desired format

% Save the figure
saveas(gcf, targetLocation);

%% Plot with respect to experimentally measured Resistance
figure;
hold on;
plot([filtered_charge_phases.cycle_no], R_estimated, '.r', 'MarkerSize', 10)
plot([filtered_charge_phases.cycle_no], Known_Res, '.k', 'MarkerSize', 10)

plot(dataTable1.cycleNumbers, dataTable1.x60__30_second,'--k');
plot(dataTable1.cycleNumbers, dataTable1.x60__1_second,'--m');
%%
% Assume 'myStructure' is the name of the structure you want to save
% Assume 'fileName' already contains the desired fileName

% Define the directory where you want to save the file
saveDirectory = folder_path;

% Ensure the directory exists, create it if it doesn't
if ~exist(saveDirectory, 'dir')
    mkdir(saveDirectory);
    disp(['Created directory: ' saveDirectory]);
end

% Construct the full file path
fullPath = fullfile(saveDirectory, fileName);

% Add .mat extension if not already present
if ~endsWith(fullPath, '.mat')
    fullPath = [fullPath '.mat'];
end

% Check if file already exists
if exist(fullPath, 'file')
    overwrite = input('File already exists. Overwrite? (y/n): ', 's');
    if ~strcmpi(overwrite, 'y')
        disp('Save operation cancelled by user.');
        return;
    end
end

% Save the structure
try
    save(fullPath,'batteryData');
    disp(['Structure successfully saved to: ' fullPath]);
catch ME
    disp('Error occurred while saving:');
    disp(ME.message);
end

% Optionally, open the directory where the file was saved
openDir = input('Open the directory where the file was saved? (y/n): ', 's');
if strcmpi(openDir, 'y')
    if ispc
        winopen(saveDirectory);
    elseif ismac
        system(['open "' saveDirectory '"']);
    elseif isunix
        system(['xdg-open "' saveDirectory '"']);
    else
        
        disp('Unable to open directory on this operating system.');
    end
end