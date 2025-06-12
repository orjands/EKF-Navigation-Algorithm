% close all; % Optional: Uncomment to close all existing figures before running.

%% === Parameters (adjust if needed) ===
% Sets up the physical constants and conversion parameters for your drive.
% - gear_ratio: Total ratio from motor to wheel
% - wheel_radius: Wheel radius in meters
% - Motor_Kv: Motor speed constant [RPM/V]
% - Kt: Torque constant [Nm/A], converts current to torque
% - gearbox_efficiency: Fraction of mechanical power preserved by gearbox (20%)
% - motor_const: Additional fudge factor for motor efficiency

gear_ratio = 16;
wheel_radius = 0.043;          % meters
const = 60/(2*pi);             % ≈9.549
Motor_Kv = 470;                % RPM/V
Kt = 60 / (2 * pi * Motor_Kv); % [Nm/A]
gearbox_efficiency = 0.2;      % Gearbox efficiency (0.2 = 20%)
motor_const = 0.95;            % Efficiency fudge (optional for Kt method)

%% === Belt 1 Calculations ===
v1 = voltage1(1:end);
i1 = current1(1:end);
rpm1 = belt_1_rpm(1:end);

Power_Belt1 = v1 .* i1;  % [W]

% Most reliable (Kt) torque model:
torque_motor1_model = Kt * i1 * motor_const;  % [Nm]
force1_model = (torque_motor1_model * gear_ratio * gearbox_efficiency) ./ wheel_radius;  % [N]

%% === Belt 2 Calculations ===
v2 = voltage2(1:end);
i2 = current2(1:end);
rpm2 = belt_2_rpm(1:end);

Power_Belt2 = v2 .* i2;  % [W]
torque_motor2_model = Kt * i2 * motor_const;  % [Nm]
force2_model = (torque_motor2_model * gear_ratio * gearbox_efficiency) ./ wheel_radius; % [N]

%% === Smoothing ===
% Applies moving average smoothing (window of 50 samples) to force and power.
% Also applies a zero-phase Butterworth filter for a smoother estimate with zero phase shift.

win = 50;  % samples (~1 second at 50Hz)
F1_smooth = movmean(force1_model, win);
F2_smooth = movmean(force2_model, win);
P1_smooth = movmean(Power_Belt1, win);
P2_smooth = movmean(Power_Belt2, win);

d1 = designfilt("lowpassiir", "FilterOrder", 12, ...
    "HalfPowerFrequency", 0.15, "DesignMethod", "butter");
F1_forback_smooth = filtfilt(d1, force1_model);
F2_forback_smooth = filtfilt(d1, force2_model);

label_size = 20;
legend_size = 15;
ref_size = 14;

%% === Figure 1: Comparison of raw and smoothed force estimates using moving average ===
figure('Name', 'Figure 1: Comparison of raw and smoothed force estimates using moving average', ...
    'NumberTitle', 'off');
plot(time, force1_model,    ':', 'Color', [0.7 0.7 0.9]); hold on;
plot(time, F1_smooth,      '-', 'LineWidth', 1.5, 'Color', [0 0.4 0.8]);
plot(time, force2_model,    ':', 'Color', [0.9 0.7 0.7]);
plot(time, F2_smooth,      '-', 'LineWidth', 1.5, 'Color', [0.8 0 0]);
set(gca, 'FontSize', axis_size);

xlabel('Time [s]', 'FontSize', label_size);
ylabel('Force [N]', 'FontSize', label_size);
legend({'Belt 1 (Right)', 'Belt 1 (Smoothed)', 'Belt 2 (Left)', 'Belt 2 (Smoothed)'}, 'FontSize', legend_size);
title('Estimated Belt Forces', 'FontSize', title_size);
grid on;

%% === Figure 2: Comparison of raw and zero-phase filtered force estimates ===
figure('Name', 'Figure 2: Comparison of raw and zero-phase filtered force estimates', ...
    'NumberTitle', 'off');
plot(time, force1_model,    ':', 'Color', [0.7 0.7 0.9]); hold on;
plot(time, F1_forback_smooth,      '-', 'LineWidth', 1.5, 'Color', [0 0.4 0.8]);
plot(time, force2_model,    ':', 'Color', [0.9 0.7 0.7]);
plot(time, F2_forback_smooth,      '-', 'LineWidth', 1.5, 'Color', [0.8 0 0]);
set(gca, 'FontSize', axis_size);

xlabel('Time [s]', 'FontSize', label_size);
ylabel('Force [N]', 'FontSize', label_size);
legend({'Belt 1 (Kt-based)', 'Belt 1 (zero-phase filt)', 'Belt 2 (Kt-based)', 'Belt 2 (zero-phase filt)'}, 'FontSize', legend_size);
title('Estimated Belt Forces – Kt Model (Zero-Phase)', 'FontSize', title_size);
grid on;

%% === Figure 3: Comparison of raw and smoothed power estimates ===
figure('Name', 'Figure 3: Comparison of raw and smoothed power estimate', ...
    'NumberTitle', 'off');
plot(time, Power_Belt1,    ':', 'Color', [0.7 0.7 0.9]); hold on;
plot(time, P1_smooth,      '-', 'LineWidth', 1.5, 'Color', [0 0.4 0.8]);
plot(time, Power_Belt2,    ':', 'Color', [0.9 0.7 0.7]);
plot(time, P2_smooth,      '-', 'LineWidth', 1.5, 'Color', [0.8 0 0]);
set(gca, 'FontSize', axis_size);

xlabel('Time [s]', 'FontSize', label_size);
ylabel('Power [W]', 'FontSize', label_size);
legend({'Belt 1 raw', 'Belt 1 smooth', 'Belt 2 raw', 'Belt 2 smooth'}, 'FontSize', legend_size);
title('Smoothed Estimated Belt Power', 'FontSize', title_size);
grid on;
