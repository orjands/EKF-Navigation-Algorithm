%% Velocity Error Calculations – EKF Tracking Accuracy under Varying Currents
% This script compares the tracking errors in surge velocity (u) and heading 
% angle (ψ) for the Remora Rover under two current conditions: no current 
% (V_c = 0.0 m/s) and moderate current (V_c = 0.5 m/s). The data is precomputed 
% using EKF simulations and visualized to evaluate how ocean currents influence 
% navigation performance.

% Note: Run 'EKF_SIMULATOR.m' first to ensure all necessary data is loaded.


clear;               % Clears all params in Workspace
% close all;         % Optional: Uncomment to close all existing figures before running.

load('u_tilde_small.mat')
load('psi_tilde_small.mat')
load('psi_tilde_large.mat')
load('u_tilde_large.mat')
load('time.mat')

%% ===================== Parameters =====================
v_c_small = 0;
v_c_large = 0.5;

switchTimes  = [50,   110,  135,   192]; % For Plotting
switchValues = [0, (0 + pi/2), (pi/2 + pi/2), (pi + pi/2), 2*pi]; % For Plotting

label_size = 20;
axis_size = 15;
title_size = 23;
legend_size = 15;
ref_size = 15;
legend_size_F3_and_4 = 12;
grayColor = [0.25 0.25 0.25]; % (Dark Gray)

%% ===================== Figure: Tracking Errors for Small vs Large Current =====================
figure('Name','Tracking Errors: Small vs Large Current','NumberTitle','off')

legend_str_small = sprintf('Current: V_c = %.2f m/s', v_c_small);
legend_str_large = sprintf('Current: V_c = %.2f m/s', v_c_large);

% --- Top plot: Surge speed tracking error
subplot(2,1,1)
plot(time, u_tilde_small, 'b-', 'LineWidth',1.5); hold on;
plot(time, u_tilde_large, 'r--', 'LineWidth',2.0);
set(gca, 'FontSize', axis_size);
ylabel('[m/s]','FontSize', label_size);
title('Tracking errors for surge speed (u) and heading (\psi) based on current', 'FontSize', title_size);
legend(legend_str_small, legend_str_large, 'Location','northeast','FontSize', legend_size);
ylim([-0.05 0.05]);
xlim([0 218]);
grid on; box on;
for k = 1:numel(switchTimes)
    labelStr = ['\psi_{ref} = ' num2str(switchValues(k+1), '%.2f') ' rad'];
    xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, ...
        'HandleVisibility', 'off', ...
        'Label', labelStr, ...
        'LabelVerticalAlignment', 'bottom', ...
        'LabelHorizontalAlignment', 'right', ...
        'LabelOrientation', 'horizontal', 'FontSize', ref_size);
end

% --- Bottom plot: Heading tracking error
subplot(2,1,2)
plot(time, psi_tilde_small, 'b-', 'LineWidth',1.5); hold on;
plot(time, psi_tilde_large, 'r--', 'LineWidth',2.0);
set(gca, 'FontSize', axis_size);
xlabel('Time [s]','FontSize',label_size);
ylabel('[rad]','FontSize',label_size);

legend(legend_str_small, legend_str_large, 'Location','northeast','FontSize', legend_size);
ylim([-0.2 0.2]);
xlim([0 218]);
grid on; box on;
for k = 1:numel(switchTimes)
    labelStr = ['\psi_{ref} = ' num2str(switchValues(k+1), '%.2f') ' rad'];
    xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, ...
        'HandleVisibility', 'off', ...
        'Label', labelStr, ...
        'LabelVerticalAlignment', 'bottom', ...
        'LabelHorizontalAlignment', 'right', ...
        'LabelOrientation', 'horizontal', 'FontSize', ref_size);
end