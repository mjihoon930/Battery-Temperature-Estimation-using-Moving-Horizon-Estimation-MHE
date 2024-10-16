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