%% EKF Loop for Test Rig Data
% Executes the EKF algorithm using data from the test rig experiments.
% Generates and displays all relevant plots for analysis.

% Note: Run 'FILE_Load_RIG.m' first to ensure all necessary data is loaded.
% Note: Include the folder WMM2025COF to path before running, for the code to work.

% close all; % Optional: Uncomment to close all existing figures before running.


%% ===================== Parameters =====================
num_steps = length(time);
dt = 0.1;

r_net = 0.205;
l = 0.13;

lat_test = 63.41;
lon_test = 10.41;
[~, ~, ~, inclination, ~] = ...
    wrldmagm(0, lat_test, lon_test, decyear(2025,6,1), 'Custom', 'WMM.COF');
theta = deg2rad(inclination);

T_encoder_speed = [1/2,  1/2; 1/(2*l), -1/(2*l)];

%% ===================== EKF Initialization =====================
x_hat = [0; 0; pi/2]; % Initial state (alpha, D, psi)
x_hat_store = zeros(3, num_steps);
x_hat_store(:, 1) = x_hat;

P = eye(3) * 0.1;   % Initial covariance
Q = eye(3) * 0.01;  % Process noise covariance matrix
R = eye(7) * 0.1;   % Measurement noise covariance matrix

z_meas_store   = zeros(7, num_steps);
y_hat_store    = zeros(7, num_steps);
x_hat_pred_store = zeros(3, num_steps);
input          = zeros(2, num_steps);

%% ===================== EKF Loop =====================
for i = 1:num_steps-1
    % -- Use speeds (u, r) from input, already provided in base workspace --
    input(:, i) = [u(i); r(i)];

    % -- EKF prediction step --
    A = state_jacobian(x_hat, input(:, i), dt, r_net);
    B = input_jacobian(x_hat, dt, r_net);
    P_pred = A * P * A' + Q;
    x_hat_pred = transition_model(x_hat, input(:, i), dt, r_net);
    x_hat_pred_store(:, i) = x_hat_pred;

    % -- Predicted measurements --
    y_hat = measurement_model(x_hat_pred, theta);
    y_hat_store(:, i) = y_hat;

    % -- Build measurement vector from real data --
    z_meas = [mag1_x(i); mag1_y(i); mag1_z(i); mag2_x(i); mag2_y(i); mag2_z(i); depth_meas(i)];
    z_meas_store(:, i) = z_meas;

    y_tilde = z_meas - y_hat;

    % -- Measurement Jacobian --
    H = measurement_jacobian(x_hat_pred, theta);

    % -- EKF correction --
    [x_hat, P] = EKF_correction(x_hat_pred, P_pred, z_meas, R, H, theta);

    x_hat_store(:, i+1) = x_hat;
end

%% ===================== Titles for each state =====================
state_titles = {'Alpha (\alpha)', 'Depth (D)', 'Heading (\psi)'};
y_labels = {'[rad]', '[m]', '[rad]'};

%% ========== Figure 1: EKF State Estimates ==========
figure('Name','Figure 1: EKF State Estimates','NumberTitle','off');
for j = 1:3
    subplot(3,1,j); hold on; grid on;
    if j == 1
        plot(time, (alpha_ref(:)),          'k--', 'LineWidth', 1.5);
        plot(time, (x_hat_pred_store(j,:)), 'b:', 'LineWidth', 1.2);
        plot(time, (x_hat_store(j,:)),      'r-', 'LineWidth', 1.5);
        set(gca, 'FontSize', axis_size);
        ylabel('[rad]', 'FontSize', label_size);
        legend({'\alpha_{ref}', 'EKF Prediction', 'EKF Estimate'}, ...
            'FontSize', legend_size_F3_and_4, 'Location', 'best');
    elseif j == 3
        plot(time(1:end-1), (psi_ref(1:end-1)),            'k--', 'LineWidth', 1.5);        
        plot(time(1:end-1), (x_hat_pred_store(j,1:end-1)), 'b:', 'LineWidth', 1.2);
        plot(time(1:end-1), (x_hat_store(j,1:end-1)),      'r-', 'LineWidth', 1.5);
        set(gca, 'FontSize', axis_size);
        ylabel(y_labels(j), 'FontSize', label_size);
        xlabel('Time [s]', 'FontSize', label_size);
        legend({'\psi_{ref}', 'EKF Prediction', 'EKF Estimate'}, ...
            'FontSize', legend_size_F3_and_4, 'Location', 'best');
    elseif j == 2
        plot(time(1:end-1), D_ref(1:end-1),              'k--', 'LineWidth', 1.5)
        plot(time(1:end-1), x_hat_pred_store(j,1:end-1), 'b:', 'LineWidth', 1.2);
        plot(time(1:end-1), x_hat_store(j,1:end-1),      'r-', 'LineWidth', 1.5);
        set(gca, 'FontSize', axis_size);
        ylabel(y_labels{j}, 'FontSize', label_size);
        legend({'D_{ref}', 'EKF Prediction', 'EKF Estimate'}, ...
            'FontSize', legend_size_F3_and_4, 'Location', 'best');
    end
    title(state_titles{j}, 'FontSize', title_size);
end

%% ========== Figure 2: Sensor Measurements vs. EKF Predictions ==========
figure('Name','Figure 2: Sensor Measurements vs. EKF Predictions','NumberTitle','off');
mag_titles = { ...
    'Mag 1: x - axis', ...
    'Mag 1: y - axis', ...
    'Mag 1: z - axis', ...
    'Mag 2: x - axis', ...
    'Mag 2: y - axis', ...
    'Mag 2: z - axis'  ...
};
y_labels_sensor = repmat({'Normalized'}, 1, 6);

for axis = 1:3
    % Left column: Mag1
    subplot(3,2,2*axis-1); hold on; grid on;
    plot(time(1:end-1), z_meas_store(axis,1:end-1), 'b', 'LineWidth', 1.5);
    plot(time(1:end-1), y_hat_store(axis,1:end-1), 'g', 'LineWidth', 1.5);
    set(gca, 'FontSize', axis_size);
    ylabel(y_labels_sensor{axis}, 'FontSize', label_size);
    if axis == 3, xlabel('Time (s)', 'FontSize', label_size); end
    title(mag_titles{axis}, 'FontSize', title_size);
    legend({'Measurement', 'EKF Prediction'}, 'FontSize', legend_size, 'Location', 'best');

    % Right column: Mag2
    subplot(3,2,2*axis); hold on; grid on;
    plot(time(1:end-1), z_meas_store(axis+3,1:end-1), 'b', 'LineWidth', 1.5);
    plot(time(1:end-1), y_hat_store(axis+3,1:end-1), 'g', 'LineWidth', 1.5);
    set(gca, 'FontSize', axis_size);
    % ylabel(y_labels_sensor{axis+3}, 'FontSize', label_size);
    if axis == 3, xlabel('Time (s)', 'FontSize', label_size); end
    title(mag_titles{axis+3}, 'FontSize', title_size);
    legend({'Measurement', 'EKF Prediction'}, 'FontSize', legend_size, 'Location', 'best');
end

%% ========== Figure 3: u and r over time (from speeds) ==========
u_est = input(1, :);
r_est = input(2, :);

figure('Name','Figure 3: u and r over time','NumberTitle','off');
subplot(2,1,1); hold on; grid on;
plot(time, u_est, 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('[m/s]', 'FontSize', label_size);
title('Surge velocity (u)', 'FontSize', title_size);

subplot(2,1,2); hold on; grid on;
plot(time, r_est, 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('[rad/s]', 'FontSize', label_size);
xlabel('Time (s)', 'FontSize', label_size);
title('Heading rate (r)', 'FontSize', title_size);

%% ========== Figure 4: Estimated right and left belt speeds ==========
belt1_speed_est = zeros(1,num_steps);
belt2_speed_est = zeros(1,num_steps);

for i = 1:num_steps
    belts2 = input(:,i); % input = [u; r]
    belt_speeds    = T_encoder_speed \ belts2;
    belt1_speed_est(i) = belt_speeds(1);
    belt2_speed_est(i) = belt_speeds(2);
end

figure('Name','Figure 4: Estimated right and left belt speeds','NumberTitle','off');
subplot(2,1,1); hold on; grid on;
plot(time, belt1_speed_est, 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('[m/s]', 'FontSize', label_size);
title('Belt 1 (Right)', 'FontSize', title_size);

subplot(2,1,2); hold on; grid on;
plot(time, belt2_speed_est, 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('[m/s]', 'FontSize', label_size);
xlabel('Time (s)', 'FontSize', label_size);
title('Belt 2 (Left)', 'FontSize', title_size);

%% ========== Figure 5: Alpha, D, and Psi error over time ==========
% Compute errors
alpha_est_deg = (x_hat_store(1,:));
psi_est_deg   = (x_hat_store(3,:));
alpha_ref_deg = (alpha_ref(:)');
psi_ref_deg   = (psi_ref(:)');
D_est         = x_hat_store(2,:);

D_error       = D_est - D_ref(:)';
alpha_error_deg = alpha_est_deg - alpha_ref_deg;
psi_error_deg   = psi_est_deg - psi_ref_deg;

figure('Name','Figure 5: Alpha, D, and Psi error over time','NumberTitle','off');
subplot(3,1,1); hold on; grid on;
plot(time, alpha_error_deg, 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('[rad]', 'FontSize', label_size);
title('Alpha Estimation Error (\alpha_{EKF} - \alpha_{ref})', 'FontSize', title_size);

subplot(3,1,2); hold on; grid on;
plot(time, D_error, 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('[m]', 'FontSize', label_size);
title('D Estimation Error (D_{EKF} - D_{ref})', 'FontSize', title_size);

subplot(3,1,3); hold on; grid on;
plot(time, psi_error_deg, 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('[rad]', 'FontSize', label_size);
xlabel('Time (s)', 'FontSize', label_size);
title('Psi Estimation Error (\psi_{EKF} - \psi_{ref})', 'FontSize', title_size);


%% ===================== EKF Functions =====================

function x_next = transition_model(states, input, dt, r_net)
    x_hat_dot = syst_dynamics(states, input, r_net);
    x_next = states + dt * x_hat_dot;
end

function x_hat_dot = syst_dynamics(states, input, r_net) 
    psi = states(3);
    u = input(1);
    r = input(2);
    alpha_dot = u * sin(-psi) / r_net;
    D_dot     = u * cos(-psi);
    psi_dot   = r;
    x_hat_dot = [alpha_dot; D_dot; psi_dot];
end

function z = measurement_model(states, theta)
    alpha = states(1); D = states(2); psi = states(3);
    mag1 = [-cos(alpha)*cos(theta); 
             cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta); 
             cos(psi)*sin(theta) + sin(alpha)*cos(theta)*sin(psi)];
    mag2 = [-cos(alpha)*cos(theta); 
             cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta); 
             cos(psi)*sin(theta) + sin(alpha)*cos(theta)*sin(psi)];
    depth = D;
    z = [mag1; mag2; depth];
end

function A = state_jacobian(states, input, dt, r_net)
    psi = states(3);
    u = input(1);
    A = eye(3);
    A(1,3) = dt * u * (cos(-psi)/r_net);
    A(2,3) = -dt * u * sin(-psi);
end

function B = input_jacobian(states, dt, r_net)
    psi = states(3);
    B = dt * [sin(-psi)/r_net, 0; 
              cos(-psi),            0; 
              0,                    1];
end

function H = measurement_jacobian(states, theta)
    alpha = states(1);
    psi   = states(3);
    H = zeros(7, 3);

    % Mag1 rows
    H(1, :) = [sin(alpha)*cos(theta), 0, 0];
    H(2, :) = [cos(alpha)*cos(psi)*cos(theta), 0, - cos(psi)*sin(theta) - sin(alpha)*cos(theta)*sin(psi)];
    H(3, :) = [cos(alpha)*cos(theta)*sin(psi), 0, cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta)];
    % Mag2 rows (identical to Mag1 here)
    H(4, :) = H(1, :);
    H(5, :) = H(2, :);
    H(6, :) = H(3, :);

    % Depth row: only depends on D (2nd state)
    H(7, 2) = 1;
end

function [x_hat_corr, P_corr] = EKF_correction(x_hat_pred, P_pred, z_meas, R, H, theta)
    y_tilde = z_meas - measurement_model(x_hat_pred, theta);
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    x_hat_corr = x_hat_pred + K * y_tilde;
    P_corr = (eye(size(P_pred)) - K * H) * P_pred;
end
