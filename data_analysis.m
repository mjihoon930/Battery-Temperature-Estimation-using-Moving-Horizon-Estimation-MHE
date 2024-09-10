%% This script is created for processing the data from the battery pack or cell to
%% get .mat format of V, I, T, along with cycling number and ambient temperature
clear all
close all
clc

%% Load Test File
folder_path='C:\Users\nqa5412\OneDrive - The Pennsylvania State University\Documents\Battery-Temperature-Estimation-using-Moving-Horizon-Estimation-MHE\Data\14226830';
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

desiredSamplingRate = 1/max(diff(dataTable.time_s)); % 1 Hz, for example. Adjust as needed.
[time_s, Current_A, Voltage_V, Temperature_C] = resampleData(dataTable, desiredSamplingRate);

% Create a structure to hold battery data
batteryData.Current = Current_A;
batteryData.Voltage_V = Voltage_V;
batteryData.Time_s = time_s;
batteryData.Temperature_C = Temperature_C;

fd;
plot(batteryData.Time_s,batteryData.Current );
fd;
plot(batteryData.Time_s,batteryData.Voltage_V );


%% Initialize variables
chargePhases = struct();  % Structure to store all charge phases
isCharging = false;
currentPhase = [];
phaseIndex = 1;
startTime = 0;  % Variable to store the start time of the current charge phase
Res=[];
Index=[];
% Loop through the data to identify and separate charge phases
for i = 1:height(batteryData.Current)
    if batteryData.Current(i) < 0  % Assuming negative current indicates charging
        if ~isCharging
            % Start of a new charge phase
            if ~isempty(currentPhase)
                chargePhases(phaseIndex).Time_s = currentPhase(:, 1);
                chargePhases(phaseIndex).Current_A = currentPhase(:, 2);
                chargePhases(phaseIndex).Voltage_V = currentPhase(:, 3);
                chargePhases(phaseIndex).Temperature_C = currentPhase(:, 4);
                 chargePhases(phaseIndex).T_amb=min(chargePhases(phaseIndex).Temperature_C);
                phaseIndex = phaseIndex + 1;
            end
            currentPhase = [];  % Reset current phase
            startTime = batteryData.Time_s(i);  % Reset the start time
            isCharging = true;
            if i>1
                if (abs(batteryData.Current(i))) > 1.5 & batteryData.Voltage_V(i)>3.5
                    Res=[Res;abs((batteryData.Voltage_V(i)-batteryData.Voltage_V(i-1))/(batteryData.Current(i)-batteryData.Current(i-1)))];
                    Index=[Index, phaseIndex];
                    Resistance(phaseIndex)=abs((batteryData.Voltage_V(i)-batteryData.Voltage_V(i-1))/(batteryData.Current(i)-batteryData.Current(i-1)));
                    chargePhases(phaseIndex).Resistance=Resistance(phaseIndex);
                    chargePhases(phaseIndex).Charge_start=i;
                end
            end
        end
        % Add the current row to the current charge phase, adjusting time to start from 0
        adjustedTime = batteryData.Time_s(i) - startTime;
        currentPhase = [currentPhase; adjustedTime, batteryData.Current(i), batteryData.Voltage_V(i), batteryData.Temperature_C(i)];
    else
        if isCharging
            % End of a charge phase
            chargePhases(phaseIndex).Time_s = currentPhase(:, 1);
            chargePhases(phaseIndex).Current_A = currentPhase(:, 2);
            chargePhases(phaseIndex).Voltage_V = currentPhase(:, 3);
            chargePhases(phaseIndex).Temperature_C = currentPhase(:, 4);
             chargePhases(phaseIndex).T_amb=min(chargePhases(phaseIndex).Temperature_C);
            phaseIndex = phaseIndex + 1;
            currentPhase = [];
            isCharging = false;
        end
    end
end

% If the last rows were in a charge phase, add them to chargePhases
if ~isempty(currentPhase)
    chargePhases(phaseIndex).Time_s = currentPhase(:, 1);
    chargePhases(phaseIndex).Current_A = currentPhase(:, 2);
    chargePhases(phaseIndex).Voltage_V = currentPhase(:, 3);
    chargePhases(phaseIndex).Temperature_C = currentPhase(:, 4);
    chargePhases(phaseIndex).T_amb=min(chargePhases(phaseIndex).Temperature_C);
end
%% Optional: Plot temperature growth for each charge phase
% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');
% Number of phases
numPhases = length(chargePhases);

% Create a continuous colormap
cmap = jet(numPhases);
colormap(cmap);

% Plot each phase
for j = Index
    Voltage_V = chargePhases(j).Voltage_V;
    if max(abs(chargePhases(j).Current_A)) > 1.5 && Voltage_V(1) > 3.5
        % Plot with color based on phase index
        
        plot(chargePhases(j).Time_s, chargePhases(j).Temperature_C, 'Color', cmap(j,:), 'LineWidth', 0.5);
    end
end

title('Voltage Across 847 Charge Phases');
xlabel('Time (s)');
ylabel('Current [Amps]');

set(axes1,'CLim',[1 847]);
% Create colorbar
colorbar(axes1);
% Adjust figure size for better visibility
set(gcf, 'Position', [100, 100, 1200, 800]);

hold off;

%% Optional: Plot CC change for each charge phase

% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');
% Number of phases
numPhases = length(chargePhases);

% Create a continuous colormap
cmap = jet(numPhases);
colormap(cmap);

% Plot each phase
for j = Index
    Voltage_V = chargePhases(j).Voltage_V;
    if max(abs(chargePhases(j).Current_A)) > 1.5 && Voltage_V(1) > 3.5
        % Plot with color based on phase index
        plot(chargePhases(j).Time_s, chargePhases(j).Current_A, 'Color', cmap(j,:), 'LineWidth', 0.5);
    end
end

title('Voltage Across 847 Charge Phases');
xlabel('Time (s)');
ylabel('Current [Amps]');

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
numPhases = length(chargePhases);

% Create a continuous colormap
cmap = jet(numPhases);
colormap(cmap);

% Plot each phase
for j = Index
    Voltage_V = chargePhases(j).Voltage_V;
    if max(abs(chargePhases(j).Current_A)) > 1.5 && Voltage_V(1) > 3.5
        % Plot with color based on phase index
        plot(chargePhases(j).Time_s, Voltage_V, 'Color', cmap(j,:), 'LineWidth', 0.5);
    end
end

title('Voltage Across Charge Phases');
xlabel('Time (s)');
ylabel('Voltage (V)');

set(axes1,'CLim',[1 847]);
% Create colorbar
colorbar(axes1);
% Adjust figure size for better visibility
set(gcf, 'Position', [100, 100, 1200, 800]);

hold off;

%% Calling Model

nc=5; %no_of_cycles_training

[theta, RMSE, MAE, Sys] = Thermal_Model(chargePhases(Index(1:nc)),Resistance(Index(1:nc)));
fd;
for j = Index
    R_Temper=chargePhases(j).Temperature_C;
    R_Tamb=chargePhases(j).T_amb;
    That_pr_ol = zeros(size(R_Temper));
    That_pr_ol(1) = R_Temper(1);
    for i = 2:length(R_Temper)
        That_pr_ol(i) = Sys.A * That_pr_ol(i-1) + Sys.B * [R_Tamb; Resistance(j) * chargePhases(j).Current_A(i-1)^2];
    end
chargePhases(j).PredictedTemperature_C=That_pr_ol;
plot(chargePhases(j).Time_s/3600, R_Temper, 'b', 'linewidth', 2);hold on;
    plot(chargePhases(j).Time_s/3600, That_pr_ol, 'r--', 'linewidth', 2);
    ylabel('Temperature [°C]');
    xlabel('Time [h]');

    % Calculate error metrics
    error = R_Temper - That_pr_ol;
    RMSE(j) = sqrt(mean(error.^2));
    MAE(j) = mean(abs(error));
end
batteryData.theta=theta;
batteryData.Sys=Sys;
batteryData.chargePhases=chargePhases;
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

%% Calling Luenberger Observer
Charge_Curr=3;
[Heat , R_estimated]=Luenberger_Observer(theta,batteryData, Charge_Curr)



