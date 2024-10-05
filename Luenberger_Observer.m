function [Heat, R_estimated, Known_Res] = Luenberger_Observer(theta, batteryData, Charge_Curr)
% Luenberger_Observer: Estimates heat generation and resistance using a Luenberger observer
%
% Inputs:
%   theta - Parameter vector for the system model
%   batteryData - Struct containing battery measurement data
%   Charge_Curr - Charging current threshold
%
% Outputs:
%   Heat - Estimated heat generation for each charge phase
%   R_estimated - Estimated resistance for each charge phase

%% System Model
A = [-theta(1), theta(2); 0, 0];
B = [theta(1); 0];
C = [1, 0];
D = [0];
Orig_sys = ss(A, B, C, D);

%% Observer Design Luenberger
L = place(A', C', [-0.065, -0.008]);
Aobs = A - L' * C;
Bobs = [B, L'];
Cobs = [1, 0];
Dobs = 0;
Obs_sys = ss(Aobs, Bobs, Cobs, Dobs);

%% Observer design LQG

% %% Observer Design
% n = size(A,1);  % number of states
% m = size(C,1);  % number of outputs
% 
% % Define the cost matrices
% Q = diag([0.009, 0.005]); % If you want to prioritize the first state more
% R = 1; 
% 
% % Solve the algebraic Riccati equation
% [P,~,~] = care(A',C',Q,R);
% 
% % Compute the optimal observer gain
% L = (P*C')/R;
% 
% % Observer system matrices
% Aobs = A - L*C;
% Bobs = [B, L];
% Cobs = eye(n);  % We want to observe all states
% Dobs = zeros(n, size(Bobs,2));
% 
% % Create the observer state-space model
% Obs_sys = ss(Aobs, Bobs, Cobs, Dobs);

%% Data Extraction
Current_A = batteryData.Current;
Voltage_V = batteryData.Voltage_V;
time_s = batteryData.Time_s;
Temperature_C = batteryData.Temperature_C;

%% Charge Phase Data Extraction
[charge_start, Ambient_Temp, Known_Res] = extractChargePhaseData(batteryData);

%% Plotting Setup
fig = figure('units', 'normalized', 'outerposition', [0 0 1 1]);
ax1 = subplot(2, 1, 1);
ax2 = subplot(2, 1, 2);

%% Heat Generation and Resistance Estimation
Heat_gen = 0;
AvgHeat = zeros(1, length(charge_start));
R_estimated = zeros(1, length(charge_start));

for j = 1:length(charge_start)
    % Determine index range for current charge phase
    if j < length(charge_start)
        index_pos = charge_start(j):charge_start(j+1)-1;
    else
        index_pos = charge_start(j):length(Voltage_V);
    end
    
    % Find end of charging period
    index_end = findChargingEnd(Current_A, index_pos,Charge_Curr);
    
    % Extract relevant data for current charge phase
    Time = time_s(index_pos);
    Curr = Current_A(index_pos);
    Temp = Temperature_C(index_pos);
    Tamb = Ambient_Temp(j) * ones(length(Temp), 1);
    
    % Run observer
    Aug_u = [Tamb, Temp];
    [Obs_T, ~, x] = lsim(Obs_sys, Aug_u', Time - Time(1), [Temp(1); Heat_gen(end)]);
    
    % Extract heat generation estimates
    Heat= x(:, 2);
    Ind0 = 1:index_end-index_pos(1);
    Heat_gen = x(Ind0, 2);
    
    % Estimate resistance
    Resistance_est = Heat_gen./ (Charge_Curr.^2);
    R_estimated(j) = mean(Resistance_est);
    
    % Plotting
    plotResults(ax1, ax2, Time, Temp, Tamb, x, Curr, Known_Res(j), Heat, Ind0);
end

% Finalize plot
finalizePlot(ax1, ax2);

fd;
plot(Known_Res);
hold on;
plot(R_estimated);
xlabel('Cycle number')
ylabel('Resistance')
legend('DeltaV/DeltaI at start of charge','Temperature based Resistance')

end

function [charge_start, Ambient_Temp, Known_Res] = extractChargePhaseData(batteryData)
charge_start = [];
Ambient_Temp = [];
Known_Res = [];
for j = 1:length(batteryData.chargePhases)
    if ~isempty(batteryData.chargePhases(j).Charge_start)
        charge_start = [charge_start, batteryData.chargePhases(j).Charge_start];
        Ambient_Temp = [Ambient_Temp, batteryData.chargePhases(j).T_amb];
        Known_Res = [Known_Res, batteryData.chargePhases(j).Resistance];
    end
end
end

function index_end = findChargingEnd(Current_A, index_pos,Charge_Curr)
positive_indices = find(Current_A(index_pos) < -0.99*Charge_Curr);
if any(diff(positive_indices)>1)
    positive_indices=positive_indices(1:find(diff(positive_indices)>1));
end

if ~isempty(positive_indices)
    index_end = index_pos(1) + positive_indices(end) - 1;
else
    warning('No positive current found in this charge phase.');
    index_end = index_pos(end);
end
end

function plotResults(ax1, ax2, Time, Temp,Tamb, x, Curr, Known_Res, Heat, I22)
% Temperature plot
plot(ax1, Time./3600, Temp, '--k', 'linewidth', 3);
hold(ax1, 'on');
plot(ax1, Time./3600, x(:, 1), 'r', 'linewidth', 2);

ylabel(ax1, 'Temperature (°C)');
set(ax1, 'xtick', [])
legend(ax1, 'Measured Temperature', 'Estimated Temperature');

% Heat generation plot
plot(ax2, Time./3600, (Curr.^2).*Known_Res.*ones(length(Time),1), '--k', 'linewidth', 2);
hold(ax2, 'on');
plot(ax2, Time./3600, Heat, 'r', 'linewidth', 2);
plot(ax2, Time(I22)./3600, Heat(I22), 'o', 'MarkerSize', 8, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
ylabel(ax2, 'Heat Generation [W]');
legend(ax2, 'Measured Heat Generation', 'Estimated Heat Generation', 'CC Charging Period Heat Generation');
ylim(ax2, [-0.1, 1.9]);
xlabel(ax2, 'Time [hr]');
end

function finalizePlot(ax1, ax2)
set(findall(gcf,'-property','FontSize'),'FontSize',24);
set(findall(gcf,'-property','interpreter'),'interpreter','latex')
set(findall(gcf,'-property','ticklabelinterpreter'),'ticklabelinterpreter','latex');
linkaxes([ax1,ax2],'x')
xlim1 = xlim(ax2);
xlim([floor(xlim1(1)), ceil(xlim1(end))]);
end