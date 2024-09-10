function [theta, RMSE, MAE, Sys4] = Thermal_Model(chargePhases,  Resistance)
    %% Batch least square thermal model with visualization and error metrics
    Sig1=[];
    Sig2=[];
    Z2=[];
        % Create a new figure
    figure('Position', [100, 100, 1200, 800]);
    for i=1:length(chargePhases)

    
    % Plot current
    ax1 = subplot(3,1,1); hold on; box on;
    plot(chargePhases(i).Time_s/3600, chargePhases(i).Current_A, 'linewidth', 2);hold on;
    ylabel('Current [A]');
    title('Battery Thermal Model Analysis');
    
    % Plot temperature
    ax2 = subplot(3,1,2); hold on; box on;
    plot(chargePhases(i).Time_s/3600, chargePhases(i).Temperature_C, 'linewidth', 2);hold on;
    ylabel('Temperature [°C]');
    
    % Plot heat generated
    ax3 = subplot(3,1,3); hold on; box on;
    plot(chargePhases(i).Time_s/3600, (chargePhases(i).Current_A.^2).*Resistance(i), 'linewidth', 2);
    hold on;
    ylabel('Heat Generated [W]');
    
    % Transfer function setup
    l1 = 10; l2 = 10;
    num = [1, 0];
    den = [1, l1+l2, l1*l2];
    sys = tf(num, den);
    R_Temper = chargePhases(i).Temperature_C;
    R_Tamb=chargePhases(i).T_amb;
    sys2 = tf(1, den);
    
    % System response calculations
    Z1 = lsim(sys, R_Temper-R_Temper(1), chargePhases(i).Time_s); %Applied filter on left side
    S1 = (R_Tamb-R_Temper); % Difference between ambient temperature and Cell Temperature
    S2 = Resistance(i) .* chargePhases(i).Current_A.^2; % Heat
    SS1 = lsim(sys2, S1, chargePhases(i).Time_s); %% Applied filter on right side signal
    SS2 = lsim(sys2, S2, chargePhases(i).Time_s); %% Applied filter on right side signal
    
    Z2=[Z2; Z1];

    Sig1=[Sig1; SS1];
    Sig2=[Sig2; SS2];
        clear Z1 S1 S2 SS1 SS2
    end

    H = [Sig1, Sig2];

    % Calculate m_t and PHI_t
    m_t = sqrt(1 + sum(H.^2, 2));
    PHI_t = H ./ m_t;
    
    Z = Z2 ./ m_t;
    condition_no = cond(PHI_t' * PHI_t);
    theta = PHI_t \ Z;

    % State-space model Continuous
    Amatrix = -theta(1);
    Bmatrix = [theta(1), theta(2)];
    Cmatrix = 1;
    Dmatrix = [0, 0];
    Sys3 = ss(Amatrix, Bmatrix, Cmatrix, Dmatrix);
    
    % Discretize the system
    for k=1:length(chargePhases)
    dt = mean(diff(chargePhases(k).Time_s)); % Use average time step
    Sys4 = c2d(Sys3, dt);
    Ad = Sys4.A;
    Bd = Sys4.B;
    R_Temper = chargePhases(k).Temperature_C;
    % Open Loop Prediction
    That_pr_ol = zeros(size(R_Temper));
    That_pr_ol(1) = R_Temper(1);
    for i = 2:length(R_Temper)
        That_pr_ol(i) = Ad * That_pr_ol(i-1) + Bd * [R_Tamb; Resistance(k) * chargePhases(k).Current_A(i-1)^2];
    end
    
    % Calculate error metrics
    error = R_Temper - That_pr_ol;
    RMSE = sqrt(mean(error.^2));
    MAE = mean(abs(error));
    
    % Plot predicted vs original temperature
    fd;
    plot(chargePhases(k).Time_s/3600, R_Temper, 'b', 'linewidth', 2); hold on;
    plot(chargePhases(k).Time_s/3600, That_pr_ol, 'r--', 'linewidth', 2);
    ylabel('Temperature [°C]');
    xlabel('Time [h]');
    legend('Original', 'Predicted', 'Location', 'best');
    title(sprintf('Temperature Comparison (RMSE: %.4f°C, MAE: %.4f°C)', RMSE, MAE));
    end
   
end