%% EKF Loop for Real-World Data with theta as state
% Executes the EKF algorithm using data from the real-world experiments.
% Generates and displays all relevant plots for analysis.

% Note: Run 'FILE_Load.m' first to ensure all necessary data is loaded.
% Note: Include the folder WMM2025COF to path before running, for the code to work.

% close all; % Optional: Uncomment to close all existing figures before running.


%% ===================== Parameters =====================
num_steps = length(time);
dt = 0.1;

r_net = 25.5;
l = 0.13;

lat_test = 58.221967; % THESE NEED TO BE CHANGED FOR EACH TEST
lon_test = 6.68495;   % THESE NEED TO BE CHANGED FOR EACH TEST
[XYZ, hor_intensity, declination, inclination, tot_intensity] = ... 
    wrldmagm(0, lat_test, lon_test, decyear(2025,5,8),'Custom','WMM.COF');

theta = deg2rad(inclination);
T_encoder_speed = [1/2,  1/2; 1/(2*l), -1/(2*l)];


%% ===================== EKF Initialization =====================
x_hat = [0; 0; 0; theta];
x_hat_store = zeros(4, num_steps);
x_hat_store(:, 1) = x_hat;

% ------- Covariance matrices (Tuning parameters for the EKF) -------
P = eye(4) * 0.01; % How sure you are on the initial guess (x_hat)?
P(4,4) = 1;        % Unsure about initial theta

Q = diag([0.01, ...   % Alpha
          0.01, ...   % D
          0.01, ...   % Psi
          1e-5]);     % Theta

R = diag([0.05, ...   % Mag1 x
          0.01, ...   % Mag1 y
          0.02, ...   % Mag1 z
          0.05, ...   % Mag2 x
          0.01, ...   % Mag2 y
          0.02, ...   % Mag2 z
          0.005]);    % Barometer
                             
z_meas_store = zeros(7, num_steps);
y_hat_store  = zeros(7, num_steps);
x_hat_pred_store = zeros(4, num_steps);
states_store = zeros(4, num_steps);
input = zeros(2, num_steps);


%% ===================== EKF Loop =====================
for i = 1:num_steps-1
    % --- EKF prediction step ---
    belt_speeds_meas = [belt_1_speed_wheel(i); belt_2_speed_wheel(i)];
    input(:, i)      = T_encoder_speed * belt_speeds_meas;

    A = state_jacobian(x_hat, input(:, i), dt, r_net);
    B = input_jacobian(x_hat, dt, r_net);
    P_pred = A * P * A' + Q;
    x_hat_pred = transition_model(x_hat, input(:, i), dt, r_net);
    x_hat_pred_store(:, i) = x_hat_pred;

    % --- EKF correction step ---
    y_hat_store(:, i) = measurement_model(x_hat_pred);

    % Build measurement vector from real data
    belt_meas = belt_speeds_meas;
    z_meas = [mag1_x(i); % normalized mag1 measurments
              mag1_y(i); % normalized mag1 measurments
              mag1_z(i); % normalized mag1 measurments
              mag2_x(i); % normalized mag2 measurments
              mag2_y(i); % normalized mag2 measurments
              mag2_z(i); % normalized mag2 measurments
              depth_meas(i) % meters
              ];
    z_meas_store(:, i) = z_meas;

    y_tilde = z_meas - measurement_model(x_hat_pred);
    H = measurement_jacobian(x_hat_pred);
    [x_hat, P] = EKF_correction(x_hat_pred, P_pred, z_meas, R, H);
    x_hat_store(:, i+1) = x_hat;
end

%% ========== Titles for each state ==========
state_titles = {'Position on Net (\alpha)', 'Depth (D)', 'Heading (\psi)'};
y_labels = {'[rad]', '[m]', '[rad]'};

%% ========== Figure 1: EKF State Estimates ==========
figure('Name','Figure 1: EKF State Estimates','NumberTitle','off');
for j = 1:3
    subplot(3,1,j); hold on; grid on;
    if j == 1
        plot(time(1:end-1), (alpha_ref(1:end-1)),          'k--', 'LineWidth', 2);
        plot(time(1:end-1), (x_hat_pred_store(j,1:end-1)), 'b:', 'LineWidth', 1.2);
        plot(time(1:end-1), (x_hat_store(j,1:end-1)),      'r-', 'LineWidth', 1.5);
        set(gca, 'FontSize', axis_size);
        ylabel(y_labels(j), 'FontSize', label_size);
        legend({'\alpha_{ref}' ,'EKF Prediction', 'EKF Estimate'}, ...
            'FontSize', legend_size, 'Location', 'best');
    elseif j == 3
        plot(time(1:end-1), (psi_ref(1:end-1)),            'k--', 'LineWidth', 2);
        plot(time(1:end-1), (x_hat_pred_store(j,1:end-1)), 'b:', 'LineWidth', 1.2);
        plot(time(1:end-1), (x_hat_store(j,1:end-1)),      'r-', 'LineWidth', 1.5);
        set(gca, 'FontSize', axis_size);
        ylabel(y_labels(j), 'FontSize', label_size);
        legend({'\psi_{ref}', 'EKF Prediction', 'EKF Estimate'}, ...
            'FontSize', legend_size, 'Location', 'best');
    else
        plot(time, D_ref, 'k--', 'LineWidth', 2); % Reference for alpha
        plot(time(1:end-1), x_hat_pred_store(j,1:end-1), 'b:', 'LineWidth', 1.2);
        plot(time(1:end-1), x_hat_store(j,1:end-1),      'r-', 'LineWidth', 1.5);
        set(gca, 'FontSize', axis_size);
        ylabel(y_labels{j}, 'FontSize', label_size);
        legend({'D_{ref}', 'EKF Prediction', 'EKF Estimate'}, ...
           'FontSize',legend_size,'Location','best'); 
        set(gca, 'YDir','reverse'); % Flip y-axis for depth
    end
    xlim([0 330]);
    if j == 3, xlabel('Time (s)', 'FontSize', label_size); end
    title([state_titles{j}], 'FontSize', title_size);
end

%% ========== Figure 2: Sensor Measurements vs. EKF Predictions ==========
figure('Name','Figure 2: Sensor Measurements vs. EKF Predictions','NumberTitle','off');

% Depth (Barometer)
subplot(4,2,[1 2]); hold on; grid on;
plot(time(1:end-1), z_meas_store(7,1:end-1), 'b', 'LineWidth', 1.5);
plot(time(1:end-1), y_hat_store(7,1:end-1), 'g', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size, 'YDir','reverse');
ylabel('Depth [m]', 'FontSize', label_size);
xlabel('Time (s)', 'FontSize', label_size);
title('Depth (Barometer)', 'FontSize', title_size);
legend({'Measurement', 'EKF Prediction'}, 'FontSize', legend_size, 'Location','best');
xlim([0 330]);

% Magnetometer axes
mag_labels = {'x - axis', 'y - axis', 'z - axis'};
for i = 1:3
    % Mag1 axis
    subplot(4,2,2*i+1); hold on; grid on;
    plot(time(1:end-1), z_meas_store(i,1:end-1), 'b', 'LineWidth', 1.5);
    plot(time(1:end-1), y_hat_store(i,1:end-1),  'g', 'LineWidth', 1.5);
    set(gca, 'FontSize', axis_size);
    ylabel('Normalized', 'FontSize', label_size);
    if i==1, ylim([-1, -0.2]); end
    if i==3, xlabel('Time (s)', 'FontSize', label_size); end
    title(['Mag 1: ' mag_labels{i}], 'FontSize', title_size);
    legend({'Measurement', 'EKF Prediction'}, 'FontSize', legend_size, 'Location','best');
    xlim([0 330]);

    % Mag2 axis
    subplot(4,2,2*i+2); hold on; grid on;
    plot(time(1:end-1), z_meas_store(i+3,1:end-1), 'b', 'LineWidth', 1.5);
    plot(time(1:end-1), y_hat_store(i+3,1:end-1),  'g', 'LineWidth', 1.5);
    set(gca, 'FontSize', axis_size);
    % ylabel('Normalized', 'FontSize', label_size);
    if i==3, xlabel('Time (s)', 'FontSize', label_size); end
    if i==1, ylim([-1, -0.2]); end
    title(['Mag 2: ' mag_labels{i}], 'FontSize', title_size);
    legend({'Measurement', 'EKF Prediction'}, 'FontSize', legend_size, 'Location','best');
    xlim([0 330]);
end

%% ========== Figure 2.1: Sensor Measurements vs. EKF Predictions (Only Mag) ==========
figure('Name','Magnetometers: Sensor Measurements vs. EKF Predictions','NumberTitle','off');

mag_labels = {'x - axis', 'y - axis', 'z - axis'};
for i = 1:3
    % Mag1 axis (left column)
    subplot(3,2,2*i-1); hold on; grid on;
    plot(time(1:end-1), z_meas_store(i,1:end-1), 'b', 'LineWidth', 1.5);
    plot(time(1:end-1), y_hat_store(i,1:end-1),  'g', 'LineWidth', 1.5);
    set(gca, 'FontSize', axis_size);
    ylabel('Normalized', 'FontSize', label_size);
    if i==3, xlabel('Time (s)', 'FontSize', label_size); end
    title(['Mag 1: ' mag_labels{i}], 'FontSize', title_size);
    legend({'Measurement', 'EKF Prediction'}, 'FontSize', legend_size, 'Location','best');
    xlim([0 330]);

    % Mag2 axis (right column)
    subplot(3,2,2*i); hold on; grid on;
    plot(time(1:end-1), z_meas_store(i+3,1:end-1), 'b', 'LineWidth', 1.5);
    plot(time(1:end-1), y_hat_store(i+3,1:end-1),  'g', 'LineWidth', 1.5);
    set(gca, 'FontSize', axis_size);
    ylabel('Normalized', 'FontSize', label_size);
    if i==3, xlabel('Time (s)', 'FontSize', label_size); end
    title(['Mag 2: ' mag_labels{i}], 'FontSize', title_size);
    legend({'Measurement', 'EKF Prediction'}, 'FontSize', legend_size, 'Location','best');
    xlim([0 330]);
end

%% ========== Figure 5: Estimated u and r (from encoder speeds) ==========
u_est = input(1, :);
r_est = input(2, :);

figure('Name','Figure 5: u and r over time','NumberTitle','off');
subplot(2,1,1); hold on; grid on;
plot(time(1:end-1), u_est(1:end-1), 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('[m/s]', 'FontSize', label_size);
title('Surge velocity (u)', 'FontSize', title_size);
xlim([0 330]);

subplot(2,1,2); hold on; grid on;
plot(time(1:end-1), r_est(1:end-1), 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('[rad/s]', 'FontSize', label_size);
xlabel('Time (s)', 'FontSize', label_size);
title('Yaw rate (r)', 'FontSize', title_size);
xlim([0 330]);

%% ========== Figure 6: Estimated right and left belt speeds ==========
belt1_speed_est = zeros(1,num_steps);
belt2_speed_est = zeros(1,num_steps);

for i = 1:num_steps
    belts2 = input(:,i); % input = [u; r]
    belt_speeds    = T_encoder_speed \ belts2;
    belt1_speed_est(i) = belt_speeds(1);
    belt2_speed_est(i) = belt_speeds(2);
end

figure('Name','Figure 6: Estimated right and left belt speeds','NumberTitle','off');
subplot(2,1,1); hold on; grid on;
plot(time(1:end-1), belt1_speed_est(1:end-1), 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('Belt Right (m/s)', 'FontSize', label_size);
title('Right Belt speed', 'FontSize', title_size);
xlim([0 330]);

subplot(2,1,2); hold on; grid on;
plot(time(1:end-1), belt2_speed_est(1:end-1), 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('Belt Left (m/s)', 'FontSize', label_size);
xlabel('Time (s)', 'FontSize', label_size);
title('Left Belt Speed', 'FontSize', title_size);
xlim([0 330]);

%% ========== Figure 7: Change in theta ==========
figure('Name','Figure 7: Change in theta','NumberTitle','off');
plot(time, x_hat_store(4,:), 'r', 'LineWidth', 2); grid on;
set(gca, 'FontSize', axis_size);
ylabel('[rad]', 'FontSize', label_size);
xlabel('Time (s)', 'FontSize', label_size);
title('Change in theta (\theta) over time', 'FontSize', title_size);
xlim([0 330]);
legend({'\theta'}, 'FontSize', legend_size+10, 'Location','best');

%% ===================== EKF Functions =====================
function x_next = transition_model(states, input, dt, r_net)
    x_hat_dot = syst_dynamics(states, input, r_net);
    x_next = states + dt * x_hat_dot;
end

function x_hat_dot = syst_dynamics(states, input, r_net) 
    psi = states(3);
    % theta = states(4);

    u = input(1);
    r = input(2);
    
    % --- Calculations
    alpha_dot = u * sin(-psi) / r_net;
    D_dot =     u * cos(-psi);
    psi_dot =   r;
    theta_dot = 0; %1e-9/theta;

    x_hat_dot = [alpha_dot;
                 D_dot;
                 psi_dot;
                 theta_dot];
end

function z = measurement_model(states)
    alpha = states(1); D = states(2); psi = states(3);
    theta = states(4);

    mag1 = [-cos(alpha)*cos(theta); 
             cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta); 
             cos(psi)*sin(theta) + sin(alpha)*cos(theta)*sin(psi)];
    mag2 = [-cos(alpha)*cos(theta); 
             cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta); 
             cos(psi)*sin(theta) + sin(alpha)*cos(theta)*sin(psi)];

    baro = D;

    z = [mag1; mag2; baro];
end

function A = state_jacobian(states, input, dt, r_net)
    psi = states(3);
    % theta = states(4);

    u = input(1);

    % Jacobian of the motion model w.r.t. state
    A = eye(4);

    A(1,3) = dt * u * (cos(-psi)/r_net);
    A(2,3) = -dt * u * sin(-psi);
    % A(4,4) = 1 - dt * (1/theta^2) * 1e-9;
end

function B = input_jacobian(states, dt, r_net)
    psi = states(3);
    % Jacobian of the motion model w.r.t. state
    B = dt * [sin(-psi)/r_net, 0; 
              cos(-psi),            0; 
              0,                    1;
              0,                    0];
end

function H = measurement_jacobian(states)
    alpha = states(1);
    psi   = states(3);
    theta = states(4);

    H = zeros(7, 4);

    % 3–5: Magnetometer 1 rows
    H(1, :) = [sin(alpha)*cos(theta), 0, 0, cos(alpha)*sin(theta)];
    H(2, :) = [cos(alpha)*cos(psi)*cos(theta), 0, - cos(psi)*sin(theta) - sin(alpha)*cos(theta)*sin(psi), - cos(theta)*sin(psi) - cos(psi)*sin(alpha)*sin(theta)];
    H(3, :) = [cos(alpha)*cos(theta)*sin(psi), 0, cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta), cos(psi)*cos(theta) - sin(alpha)*sin(psi)*sin(theta)];
    % 6–8: Magnetometer 2 rows
    H(4, :) = [sin(alpha)*cos(theta), 0, 0, cos(alpha)*sin(theta)];
    H(5, :) = [cos(alpha)*cos(psi)*cos(theta), 0, - cos(psi)*sin(theta) - sin(alpha)*cos(theta)*sin(psi), - cos(theta)*sin(psi) - cos(psi)*sin(alpha)*sin(theta)];
    H(6, :) = [cos(alpha)*cos(theta)*sin(psi), 0, cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta), cos(psi)*cos(theta) - sin(alpha)*sin(psi)*sin(theta)];

    % 6: Barometer → depth directly
    H(7, 2) = 1;
end

function [x_hat_corr, P_corr] = EKF_correction(x_hat_pred, P_pred, z_meas, R, H)
    y_tilde = z_meas - measurement_model(x_hat_pred);
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    x_hat_corr = x_hat_pred + K * y_tilde;
    P_corr = (eye(size(P_pred)) - K * H) * P_pred;
end