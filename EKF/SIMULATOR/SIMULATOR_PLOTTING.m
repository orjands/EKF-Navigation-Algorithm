%% Visualization of EKF Simulator Results
% This script generates plots from the EKF-based simulation environment 
% developed for Remora Robotics’ underwater net-grooming robot.

% Note: Run 'EKF_SIMULATOR.m' first to ensure all necessary data is loaded.

%% ===================== Plotting Params  =====================
% NED coordinate calculations based on SINTEF paper
N_arr = r_net_state_store .* cos(states(1, :));
E_arr = r_net_state_store .* sin(states(1, :));
D_arr = states(2, :);

time = (0:num_steps-1) * time_step;
idx = 1:(numel(time)-1);

label_size = 20;
axis_size = 15;
title_size = 23;
legend_size = 15;
ref_size = 15;
legend_size_F3_and_4 = 12;
grayColor = [0.25 0.25 0.25]; % (Dark Gray)

%% ===================== Figure 1: 3D Trajectory in NED =====================
figure('Name', 'Figure 1: 3D Trajectory in NED', 'NumberTitle', 'off');
plot3(N_arr, E_arr, D_arr, 'b', 'LineWidth', 2.0); hold on;
plot3(N_arr(1), E_arr(1), D_arr(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start point
plot3(N_arr(end), E_arr(end), D_arr(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End point
set(gca, 'FontSize', axis_size);
xlabel('North [m]', 'FontSize', label_size);
ylabel('East [m]',  'FontSize', label_size);
zlabel('Down [m]',  'FontSize', label_size);
set(gca, 'YDir', 'reverse');   % Flip y-axis so positive is East
set(gca, 'ZDir', 'reverse');   % Flip z-axis so positive is downward
title('3D Robot Trajectory in NED', 'FontSize', title_size);
legend('Trajectory', 'Start', 'End', 'FontSize', legend_size);
grid on; view(3);

%% ===================== Figure 2: Water-Relative vs Desired Surge Speed =====================
figure('Name', 'Figure 2: Water-Relative vs Desired Surge Speed', 'NumberTitle', 'off');
hold on; grid on;
plot(time(idx), u_r(idx),      'b',  'LineWidth',1.5);
plot(time(idx), u_d_store(idx),'r--','LineWidth',1.5);
set(gca, 'FontSize', axis_size);
xlabel('Time (s)', 'FontSize', label_size);
xlim([0 218]);
ylabel('[m/s]', 'FontSize', label_size);
legend('Water‐relative speed: u_r','Desired speed: u_d','Location','Best','FontSize', legend_size);
title('Water‐relative Surge Speed Tracking', 'FontSize', title_size);

%% ===================== Figure 3: EKF Estimates vs True States =====================
figure('Name', 'Figure 3: EKF Estimates vs True States', 'NumberTitle', 'off');
t = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% === 1. ψ (Heading Angle) — Full-width on top ===
nexttile([1 2]);
j = 3; % ψ
hold on; grid on;
plot(time, states(j,:), 'b', 'LineWidth', 1.2);
plot(time, x_hat_store(j,:), 'r', 'LineWidth', 1.2);
set(gca, 'FontSize', axis_size);
xlabel('Time (s)', 'FontSize', label_size); ylabel('[rad]', 'FontSize', label_size);
title('Heading (\psi)', 'FontSize', title_size);
legend('True State', 'EKF Estimate', 'FontSize', legend_size_F3_and_4);
xlim([0 218]);
ylim([-0.5 10])
for k = 1:numel(switchTimes)
    labelStr = ['\psi_{ref} = ' num2str(switchValues(k+1), '%.2f') ' rad'];
    if any(switchTimes(k) == [50 110])
        vAlign = 'top';
    else
        vAlign = 'bottom';
    end
    xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, ...
        'HandleVisibility', 'off', ...
        'Label', labelStr, ...
        'LabelVerticalAlignment', vAlign, ...
        'LabelOrientation', 'horizontal', 'FontSize', ref_size);
end

% === 2. α (Azimuth Angle) — left, middle row ===
nexttile;
j = 1;
hold on; grid on;
plot(time, states(j,:), 'b', 'LineWidth', 1.2);
plot(time, x_hat_store(j,:), 'r', 'LineWidth', 1.2);
set(gca, 'FontSize', axis_size);
% xlabel('Time (s)', 'FontSize', label_size); 
ylabel('[rad]', 'FontSize', label_size);
title('Position on Net (\alpha)', 'FontSize', title_size);
legend('True State', 'EKF Estimate', 'FontSize', legend_size_F3_and_4);
xlim([0 218]);
ylim([-1 0.1])
for k = 1:numel(switchTimes)
    alpha_val = interp1(time, states(1,:), switchTimes(k));
    labelStr = ['\alpha = ', refValueToString(alpha_val), ' rad'];
    if any(switchTimes(k) == [110 192])
        vAlign = 'top'; hAlign = 'left';
    else
        vAlign = 'bottom'; hAlign = 'right';
    end
    xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, ...
        'HandleVisibility', 'off', ...
        'Label', labelStr, ...
        'LabelVerticalAlignment', vAlign, ...
        'LabelHorizontalAlignment', hAlign, ...
        'LabelOrientation', 'horizontal', 'FontSize', ref_size);
end

% === 3. u (Surge Speed) — right, middle row ===
nexttile;
j = 4;
hold on; grid on;
plot(time, states(j,:), 'b', 'LineWidth', 1.2);
plot(time, x_hat_store(j,:), 'r', 'LineWidth', 1.2);
set(gca, 'FontSize', axis_size);
% xlabel('Time (s)', 'FontSize', label_size); 
ylabel('[m/s]', 'FontSize', label_size);
title('Surge Speed (u)', 'FontSize', title_size);
legend('True State', 'EKF Estimate', 'FontSize', legend_size_F3_and_4);
xlim([0 218]);
yline(0.3, '--', 'Color', grayColor, 'LineWidth', 1.2, ...
    'Label', 'u_{ref} = 0.3', ...
    'LabelHorizontalAlignment', 'right', ...
    'LabelVerticalAlignment', 'bottom', ...
    'FontSize', ref_size, 'HandleVisibility', 'off');
yline(-0.3, '--', 'Color', grayColor, 'LineWidth', 1.2, ...
    'Label', 'u_{ref} = -0.3', ...
    'LabelHorizontalAlignment', 'left', ...
    'LabelVerticalAlignment', 'top', ...
    'FontSize', ref_size, 'HandleVisibility', 'off');
for k = 1:numel(switchTimes)
    xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

% === 4. D (Depth) — left, bottom row ===
nexttile;
j = 2;
hold on; grid on;
plot(time, states(j,:), 'b', 'LineWidth', 1.2);
plot(time, x_hat_store(j,:), 'r', 'LineWidth', 1.2);
set(gca, 'FontSize', axis_size);
xlabel('Time (s)', 'FontSize', label_size); ylabel('[m]', 'FontSize', label_size);
title('Depth (D)', 'FontSize', title_size);
legend('True State', 'EKF Estimate', 'FontSize', legend_size_F3_and_4);
xlim([0 218]);
ylim([-20 20])
set(gca, 'YDir', 'reverse');
for k = 1:numel(switchTimes)
    xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

% === 5. r (Yaw Rate) — right, bottom row ===
nexttile;
j = 5;
hold on; grid on;
plot(time, states(j,:), 'b', 'LineWidth', 1.2);
plot(time, x_hat_store(j,:), 'r', 'LineWidth', 1.2);
set(gca, 'FontSize', axis_size);
xlabel('Time (s)', 'FontSize', label_size); ylabel('[rad/s]', 'FontSize', label_size);
title('Heading Rate (r)', 'FontSize', title_size);
legend('True State', 'EKF Estimate', 'FontSize', legend_size_F3_and_4);
xlim([0 218]);
for k = 1:numel(switchTimes)
    xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

%% ===================== Figure 3:1: EKF State Estimation Errors =====================
% figure('Name', 'Figure 3.1: EKF Estimation Errors vs True States', 'NumberTitle', 'off');
% t = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
% 
% state_labels = {'\alpha (rad)', 'D (m)', '\psi (rad)', 'u (m/s)', 'r (rad/s)'};
% y_labels = {'$\tilde{\alpha}$ [rad]', '$\tilde{D}$ [m]', '$\tilde{\psi}$ [rad]', '$\tilde{u}$ [m/s]', '$\tilde{r}$ [rad/s]'};
% titles = {'Azimuth Angle Error', 'Depth Error', 'Heading Angle Error', 'Surge Speed Error', 'Yaw Rate Error'};
% 
% legend_str = 'Estimation error (state - EKF)';
% 
% % 1. Heading angle error — full-width on top
% nexttile([1 2]);
% j = 3;
% ekf_err = states(j,:) - x_hat_store(j,:);
% plot(time, ekf_err, 'r', 'LineWidth', 1.5); grid on; hold on;
% xlabel('Time (s)', 'FontSize', label_size); 
% ylabel(y_labels{j}, 'Interpreter', 'latex', 'FontSize', label_size);
% title('Heading Angle Error', 'FontSize', label_size);
% set(gca, 'FontSize', label_size);
% legend(legend_str, 'FontSize', legend_size_F3_and_4, 'Location', 'northeast');
% xlim([0 218]);
% for k = 1:numel(switchTimes)
%     labelStr = ['\psi_{ref} = ' num2str(switchValues(k+1), '%.2f') ' rad'];
%     if any(switchTimes(k) == [50 110])
%         vAlign = 'top';
%     else
%         vAlign = 'bottom';
%     end
%     xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, ...
%         'HandleVisibility', 'off', ...
%         'Label', labelStr, ...
%         'LabelVerticalAlignment', vAlign, ...
%         'LabelOrientation', 'horizontal', 'FontSize', ref_size);
% end
% 
% % 2. Azimuth angle error — left, middle row
% nexttile;
% j = 1;
% ekf_err = states(j,:) - x_hat_store(j,:);
% plot(time, ekf_err, 'r', 'LineWidth', 1.5); grid on; hold on;
% xlabel('Time (s)', 'FontSize', label_size);
% ylabel(y_labels{j}, 'Interpreter', 'latex', 'FontSize', label_size);
% title('Azimuth Angle Error', 'FontSize', label_size);
% set(gca, 'FontSize', label_size);
% legend(legend_str, 'FontSize', legend_size_F3_and_4, 'Location', 'northeast');
% xlim([0 218]);
% for k = 1:numel(switchTimes)
%     alpha_val = interp1(time, states(1,:), switchTimes(k));
%     labelStr = ['\alpha = ', refValueToString(alpha_val), ' rad'];
%     if any(switchTimes(k) == [110 192])
%         vAlign = 'top'; hAlign = 'left';
%     else
%         vAlign = 'bottom'; hAlign = 'right';
%     end
%     xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, ...
%         'HandleVisibility', 'off', ...
%         'Label', labelStr, ...
%         'LabelVerticalAlignment', vAlign, ...
%         'LabelHorizontalAlignment', hAlign, ...
%         'LabelOrientation', 'horizontal', 'FontSize', ref_size);
% end
% 
% % 3. Surge Speed error — right, middle row
% nexttile;
% j = 4;
% ekf_err = states(j,:) - x_hat_store(j,:);
% plot(time, ekf_err, 'r', 'LineWidth', 1.5); grid on; hold on;
% xlabel('Time (s)', 'FontSize', label_size);
% ylabel(y_labels{j}, 'Interpreter', 'latex', 'FontSize', label_size);
% title('Surge Speed Error', 'FontSize', label_size);
% set(gca, 'FontSize', label_size);
% legend(legend_str, 'FontSize', legend_size_F3_and_4, 'Location', 'northeast');
% xlim([0 218]);
% for k = 1:numel(switchTimes)
%     xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, 'HandleVisibility', 'off');
% end
% 
% % 4. Depth error — left, bottom row
% nexttile;
% j = 2;
% ekf_err = states(j,:) - x_hat_store(j,:);
% plot(time, ekf_err, 'r', 'LineWidth', 1.5); grid on; hold on;
% xlabel('Time (s)', 'FontSize', label_size);
% ylabel(y_labels{j}, 'Interpreter', 'latex', 'FontSize', label_size);
% title('Depth Error', 'FontSize', label_size);
% set(gca, 'FontSize', label_size);
% legend(legend_str, 'FontSize', legend_size_F3_and_4, 'Location', 'northeast');
% xlim([0 218]);
% set(gca, 'YDir', 'reverse');
% for k = 1:numel(switchTimes)
%     xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, 'HandleVisibility', 'off');
% end
% 
% % 5. Yaw rate error — right, bottom row
% nexttile;
% j = 5;
% ekf_err = states(j,:) - x_hat_store(j,:);
% plot(time, ekf_err, 'r', 'LineWidth', 1.5); grid on; hold on;
% xlabel('Time (s)', 'FontSize', label_size);
% ylabel(y_labels{j}, 'Interpreter', 'latex', 'FontSize', label_size);
% title('Yaw Rate Error', 'FontSize', label_size);
% set(gca, 'FontSize', label_size);
% legend(legend_str, 'FontSize', legend_size_F3_and_4, 'Location', 'northeast');
% xlim([0 218]);
% for k = 1:numel(switchTimes)
%     xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, 'HandleVisibility', 'off');
% end

%% ===================== Figure 3.2: EKF Estimation Errors: Azimuth & Heading =====================
figure('Name', 'EKF Estimation Errors: Azimuth & Heading', 'NumberTitle', 'off');

% -- 1. Azimuth Angle Error
subplot(2,1,1)
azimuth_err = states(1,:) - x_hat_store(1,:);
plot(time, azimuth_err, 'r', 'LineWidth', 1.5); grid on; hold on;
set(gca, 'FontSize', axis_size);
% xlabel('Time (s)', 'FontSize', label_size);
ylabel('[rad]', 'FontSize', label_size);
title('Alpha Estimation Error (\alpha_{EKF} - \alpha_{ref})', 'FontSize', title_size);
xlim([0 218]);
ylim([-0.5 0.5]);

% -- 2. Heading Angle Error
subplot(2,1,2)
heading_err = states(3,:) - x_hat_store(3,:);
plot(time, heading_err, 'r', 'LineWidth', 1.5); grid on; hold on;
set(gca, 'FontSize', axis_size);
xlabel('Time (s)', 'FontSize', label_size);
ylabel('[rad]', 'FontSize', label_size);
title('Heading Estimation Error (\psi_{EKF} - \psi_{ref})', 'FontSize', title_size);
xlim([0 218]);
ylim([-0.5 0.5]);


%% ===================== Figure 4: Actual Sensor Measurements vs. EKF Predictions =====================
figure('Name', 'Figure 4: Actual Sensor Measurements vs. EKF Predictions', 'NumberTitle', 'off');

% Magnetometer in left column, others in right column
% Order: [Mag-X, Encoder Right; Mag-Y, Encoder Left; Mag-Z, Barometer]
mag_titles = {'Mag: x-Axis', 'Mag: y-Axis', 'Mag: z-Axis'};
mag_ylabel = {'Normalized', 'Normalized', 'Normalized'};
other_titles = {'Belt 1 (Right)', 'Belt 2 (Left)', 'Barometer (Depth Sensor)'};
other_ylabel = {'[m/s]', '[m/s]', '[m]'};

for row = 1:3
    % --- Left: Magnetometer axes ---
    subplot(3,2,2*row-1); hold on; grid on;
    mag_idx = row; % Mag1 x, y, z are 3, 4, 5 in original order
    h1 = plot(time(idx), z_meas_store(mag_idx+2, idx), 'b', 'LineWidth', 1.5);  % Measured
    h2 = plot(time(idx), y_hat_store(mag_idx+2, idx), 'g', 'LineWidth', 1.5);  % EKF Prediction
    set(gca, 'FontSize', axis_size);    
    ylabel(mag_ylabel{row}, 'FontSize', label_size);
    title(mag_titles{row}, 'FontSize', title_size);
    legend([h1 h2], {'Measurement', 'EKF Prediction'}, 'Location', 'best','FontSize',legend_size_F3_and_4);
    xlim([0 218]);
    if row == 3, xlabel('Time (s)', 'FontSize', label_size); end
    % Add ref lines/labels for Mag plots
    for k = 1:numel(switchTimes)
        labelStr = ['\psi_{ref} = ' num2str(switchValues(k+1), '%.2f') ' rad'];
        if any(switchTimes(k) == [50 110 192])
            vAlign = 'top'; hAlign = 'left';
        else
            vAlign = 'bottom'; hAlign = 'right';
        end
        xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, ...
            'HandleVisibility', 'off', ...
            'Label', labelStr, ...
            'LabelVerticalAlignment', vAlign, ...
            'LabelHorizontalAlignment', hAlign, ...
            'LabelOrientation', 'horizontal', 'FontSize', ref_size);
    end

    % --- Right: Encoders and Baro ---
    subplot(3,2,2*row); hold on; grid on;
    % Encoder 1 = 1, Encoder 2 = 2, Baro = 6
    other_idx = [1 2 6];
    h1 = plot(time(idx), z_meas_store(other_idx(row), idx), 'b', 'LineWidth', 1.5);  % Measured
    h2 = plot(time(idx), y_hat_store(other_idx(row), idx), 'g', 'LineWidth', 1.5);  % EKF Prediction
    set(gca, 'FontSize', axis_size);
    ylabel(other_ylabel{row}, 'FontSize', label_size);
    title(other_titles{row}, 'FontSize', title_size);
    legend([h1 h2], {'Measurement', 'EKF Prediction'}, 'Location', 'best','FontSize',legend_size_F3_and_4);
    xlim([0 218]);
    if row == 3, xlabel('Time (s)', 'FontSize', label_size); end
    if row == 3
        if other_idx(row) == 6
            set(gca, 'YDir', 'reverse');
        end
    end
    % Add ref lines/labels for Baro plot only
    if other_idx(row) == 6
        for k = 1:numel(switchTimes)
            labelStr = ['\psi_{ref} = ' num2str(switchValues(k+1), '%.2f') ' rad'];
            if any(switchTimes(k) == [50 110 192])
                vAlign = 'top'; hAlign = 'left';
            else
                vAlign = 'bottom'; hAlign = 'right';
            end
            xline(switchTimes(k), '--', 'Color', grayColor, 'LineWidth', 1.2, ...
                'HandleVisibility', 'off', ...
                'Label', labelStr, ...
                'LabelVerticalAlignment', vAlign, ...
                'LabelHorizontalAlignment', hAlign, ...
                'LabelOrientation', 'horizontal', 'FontSize', ref_size);
        end
    end
end


%% ===================== Figure 5: Net Radius Over Time =====================
figure('Name', 'Figure 5: Net Radius Over Time', 'NumberTitle', 'off');
plot(time, r_net_state_store, 'LineWidth', 2);
set(gca, 'FontSize', axis_size);
xlabel('Time (s)', 'FontSize', label_size);
xlim([0 218]);
ylim([(min(r_net_state_store) - 0.1) (max(r_net_state_store) + 0.1)]);
ylabel('[m]', 'FontSize', label_size);
title('Evolution of the Net Radius Over Time', 'FontSize', title_size);
grid on;

%% ===================== Figure 6: Tracking Errors based on current =====================
figure('Name','Figure 6: Tracking Errors. Current','NumberTitle','off')
legend_str = sprintf('Current: V_c = %.2f m/s', v_c);

% --- Top plot: Surge speed tracking error
subplot(2,1,1)
plot(time, u_tilde_store, 'r-', 'LineWidth', 2.0); hold on;
set(gca, 'FontSize', axis_size);
ylabel('[m/s]','FontSize',label_size);
legend(legend_str,'Location','northeast','FontSize',legend_size);
title('Tracking errors for surge speed (u) and heading (\psi)', 'FontSize', title_size);
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
plot(time, psi_tilde_store, 'r-', 'LineWidth',2.0); hold on;
set(gca, 'FontSize', axis_size);
xlabel('Time [s]','FontSize',label_size);
ylabel('[rad]','FontSize',label_size);
legend(legend_str,'Location','northeast','FontSize',legend_size);
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

%% ===================== Plotting Function =====================
function str = refValueToString(val)
    tol = 1e-6;
    if abs(val) < tol
        str = '0';
    elseif abs(val - pi/2) < tol
        str = '\pi/2';
    elseif abs(val + pi/2) < tol
        str = '-\pi/2';
    elseif abs(val - pi) < tol
        str = '\pi';
    elseif abs(val + pi) < tol
        str = '-\pi';
    else
        % Fallback: display as numeric (with two decimals)
        str = sprintf('%.2f', val);
    end
end