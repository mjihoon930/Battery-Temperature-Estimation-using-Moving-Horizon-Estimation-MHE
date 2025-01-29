function [time_s_uniform, Current_A_uniform, Voltage_V_uniform, Temperature_C_uniform] = resampleData(dataTable, desiredSamplingRate)
    % Extract original data
    time_s = dataTable.time_s;
    Current_A = -1 * (dataTable.I_mA) * 1e-3; % current positive for discharge and negative for charge
    Voltage_V = dataTable.Ecell_V;
    Temperature_C = dataTable.Temperature__C;

    % Remove duplicate time points
    [time_s_unique, uniqueIndices] = unique(time_s, 'stable');
    Current_A = Current_A(uniqueIndices);
    Voltage_V = Voltage_V(uniqueIndices);
    Temperature_C = Temperature_C(uniqueIndices);

    % Define new time vector with uniform spacing
    time_s_uniform = (time_s_unique(1):1/desiredSamplingRate:time_s_unique(end))';

    % Resample data using interpolation
    Current_A_uniform = interp1(time_s_unique, Current_A, time_s_uniform, 'linear');
    Voltage_V_uniform = interp1(time_s_unique, Voltage_V, time_s_uniform, 'linear');
    Temperature_C_uniform = interp1(time_s_unique, Temperature_C, time_s_uniform, 'linear');

    % Remove any NaN values that might occur at the edges
    validIdx = ~isnan(Current_A_uniform) & ~isnan(Voltage_V_uniform) & ~isnan(Temperature_C_uniform);
    time_s_uniform = time_s_uniform(validIdx);
    Current_A_uniform = Current_A_uniform(validIdx);
    Voltage_V_uniform = Voltage_V_uniform(validIdx);
    Temperature_C_uniform = Temperature_C_uniform(validIdx);
end