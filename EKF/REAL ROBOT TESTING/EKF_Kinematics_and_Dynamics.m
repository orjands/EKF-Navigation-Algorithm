%% EKF Loop for Real-World Data
% Executes the EKF algorithm using data from the real-world experiments.
% Generates and displays all relevant plots for analysis.

% Note: Run 'FILE_Load.m' first to ensure all necessary data is loaded.
% Note: Run 'FORCE_Calculations.m' first to ensure all necessary data is loaded.
% Note: Include the folder WMM2025COF to path before running, for the code to work.

% close all; % Optional: Uncomment to close all existing figures before running.


%% ===================== Parameters =====================
mass = 24;      % Mass (kg)
inertia = 5;    % Inertia around z-axis (kg*m^2)
Xu_dot = -2*2;  % Added mass in surge
Nr_dot = -1;    % Added inertia in yaw
Xu = -3*2;      % Linear damping in surge
Nr = -1;        % Linear damping in yaw
Xuu = -1*2;     % Nonlinear damping in surge
Nrr = -0.1;     % Nonlinear damping in yaw

sim_params = [mass, Xu, Xuu, Xu_dot, inertia, Nr, Nrr, Nr_dot]; % Test: Ideal Conditions

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
T_encoder_force = [1,  1; -l, l];


%% ===================== EKF Initialization =====================
x_hat = [0; 0; 0; 0; 0];
x_hat_store = zeros(5, num_steps);
x_hat_store(:, 1) = x_hat;

% ------- Covariance matrices (Tuning parameters for the EKF) -------
P = eye(5) * 0.1; 

% Process noise covariance matrix Q
% Q = eye(5) * 1;
% Q = diag([0.1, 0.01, 0.01, 0.1, 0.1]); 

% Measurement noise covariance matrix R.
% R = eye(9) * 0.05;
% R = diag([0.1, 0.1, 0.01, 0.01, 0.02, 0.01, 0.01, 0.02, 0.005]);

% Test 1: Trying to better alpha estimates:
% lower Q(1,1) % Trust the mathematical model 
Q = diag([0.0001, ... % Alpha
          0.01, ...   % D
          0.01, ...   % Psi
          0.1, ...    % u
          0.1]);      % r
% increase R(3,3), R(6,6) % Don't trust the measurments
R = diag([0.1, ...    % Right Belt
          0.1, ...    % Left Belt
          0.5, ...    % Mag1 x
          0.01, ...   % Mag1 y
          0.02, ...   % Mag1 z
          0.5, ...    % Mag2 x
          0.01, ...   % Mag2 y
          0.02, ...   % Mag2 z
          0.005]);    % Barometer
                    
z_meas_store = zeros(9, num_steps);
y_hat_store  = zeros(9, num_steps);
x_hat_pred_store = zeros(5, num_steps);
states_store = zeros(5, num_steps);
input = zeros(2, num_steps);


%% ===================== EKF Loop =====================
for i = 1:num_steps-1
    % --- EKF prediction step ---
    A = state_jacobian(x_hat, dt, r_net, sim_params);
    B = input_jacobian(sim_params, dt);
    
    % Combine belt forces to estimate surge and yaw torques
    input(:, i) = T_encoder_force * [F1_smooth(i); F2_smooth(i)];

    x_hat_pred = transition_model(x_hat, input(:, i), dt, sim_params, r_net);
    x_hat_pred_store(:, i) = x_hat_pred;
    P_pred = A * P * A' + Q;

    % --- EKF correction step ---
    y_hat_store(:, i) = measurement_model(x_hat_pred, theta, T_encoder_speed);

    % Build measurement vector from real data
    belt_speeds_meas = [belt_1_speed(i); belt_2_speed(i)];
    belt_meas = belt_speeds_meas;

    z_meas = [belt_meas; % m/s
              mag1_x(i); % normalized mag1 measurments
              mag1_y(i); % normalized mag1 measurments
              mag1_z(i); % normalized mag1 measurments
              mag2_x(i); % normalized mag2 measurments
              mag2_y(i); % normalized mag2 measurments
              mag2_z(i); % normalized mag2 measurments
              depth_meas(i) % meters
              ];
    z_meas_store(:, i) = z_meas;

    y_tilde = z_meas - measurement_model(x_hat_pred, theta, T_encoder_speed);
    H = measurement_jacobian(x_hat_pred, theta, T_encoder_speed);
    [x_hat, P] = EKF_correction(x_hat_pred, P_pred, z_meas, R, H, theta, T_encoder_speed);
    x_hat_store(:, i+1) = x_hat;
end


%% ========== Titles for each state ==========
y_labels = {'[rad]', '[m]', '[rad]', '[m/s]', '[rad/s]'};
state_titles = {'Position on Net (\alpha)', 'Depth (D)', 'Heading (\psi)', 'Surge speed (u)', 'Heading rate (r)'};


%% ========== Figure 1: EKF State Estimates ==========
figure('Name','Figure 1: EKF State Estimates','NumberTitle','off');
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
plot_order = [3 1 4 2 5]; % psi, alpha, u, D, r

for p = 1:5
    if p == 1
        nexttile([1 2]); j = 3; % psi full-width
    elseif p == 2
        nexttile; j = 1; % alpha left, middle
    elseif p == 3
        nexttile; j = 4; % u right, middle
    elseif p == 4
        nexttile; j = 2; % D left, bottom
    else
        nexttile; j = 5; % r right, bottom
    end

    hold on; grid on;

    % --- Add psi_ref for heading plot only
    if p == 1
        plot(time, psi_ref, 'k--', 'LineWidth', 1.6); % Reference for heading
    end

    % --- Add alpha_ref for azimuth plot only
    if p == 2
        plot(time, alpha_ref, 'k--', 'LineWidth', 1.6); % Reference for alpha
    end

    if p == 4
        plot(time, D_ref, 'k--', 'LineWidth', 1.6); % Reference for alpha
        set(gca, 'YDir','reverse'); 
    end

    plot(time(1:end-1), x_hat_pred_store(j,1:end-1), 'b:', 'LineWidth', 1.2);
    plot(time(1:end-1), x_hat_store(j,1:end-1),     'r-', 'LineWidth', 1.5);
    set(gca, 'FontSize', axis_size);
    xlim([0 330]);
    ylabel(y_labels{j}, 'FontSize', label_size);
    if p > 2
        xlabel('Time (s)', 'FontSize', label_size);
    end
    title(state_titles{j}, 'FontSize', title_size);
    % Legends
    if p == 1
        legend({'\psi_{ref}', 'EKF Prediction', 'EKF Estimate'}, ...
           'FontSize',legend_size,'Location','best');
    elseif p == 2
        legend({'\alpha_{ref}', 'EKF Prediction', 'EKF Estimate'}, ...
           'FontSize',legend_size,'Location','best');
    elseif p == 4
        legend({'D_{ref}', 'EKF Prediction', 'EKF Estimate'}, ...
           'FontSize',legend_size,'Location','best');        
    else
        legend({'EKF Prediction', 'EKF Estimate'}, ...
           'FontSize', legend_size, 'Location', 'best');
    end
end

%% ========== Figure 3: Sensor Measurements vs. EKF Predictions ==========
figure('Name','Figure 3: Sensor Measurements vs. EKF Predictions','NumberTitle','off');

plot_map = [1 3 6;    % Row 1: Encoder1 | Mag1 x | Mag2 x
            2 4 7;    % Row 2: Encoder2 | Mag1 y | Mag2 y
            9 5 8]';  % Row 3: Baro     | Mag1 z | Mag2 z

sensor_titles = { ...
    'Encoder: Belt 1', ...   % 1
    'Encoder: Belt 2', ...   % 2
    'Mag 1: x', ...          % 3
    'Mag 1: y', ...          % 4
    'Mag 1: z', ...          % 5
    'Mag 2: x', ...          % 6
    'Mag 2: y', ...          % 7
    'Mag 2: z', ...          % 8
    'Depth (Barometer)' ...  % 9
    };
y_labels_sensor = { ...
    'm/s', 'm/s', ...
    'Normalized', 'Normalized', 'Normalized', ...
    'Normalized', 'Normalized', 'Normalized', ...
    'Depth [m]' ...
    };

for row = 1:3
    for col = 1:3
        idx = plot_map(row, col);
        subplot(3, 3, (col-1)*3 + row); hold on;
        plot(time, z_meas_store(idx,:), 'b', 'LineWidth', 1.5);
        plot(time, y_hat_store(idx,:), 'g', 'LineWidth', 1.5);
        set(gca, 'FontSize', axis_size);
        xlim([0 330]);
        ylabel(y_labels_sensor{idx}, 'FontSize', label_size);
        xlabel('Time (s)', 'FontSize', label_size);
        title(sensor_titles{idx}, 'FontSize', title_size);
        legend({'Sensor Measurement', 'EKF Prediction'}, 'FontSize', legend_size, 'Location','best');
        if idx == 9, set(gca, 'YDir','reverse'); end
        grid on;
    end
end

%% ========== Figure 3.1 - 3.3: Sensor Measurements vs. EKF Predictions ==========
figure('Name','Belt Speeds: Sensor vs EKF','NumberTitle','off');
% --- Belt 1 ---
subplot(2,1,1); hold on; grid on;
plot(time, z_meas_store(1,:), 'b', 'LineWidth', 1.5);      % Belt 1 (Sensor)
plot(time, y_hat_store(1,:), 'g', 'LineWidth', 1.5);       % Belt 1 (EKF)
set(gca, 'FontSize', axis_size);
xlim([0 330]);
ylabel('Velocity [m/s]', 'FontSize', label_size);
title('Belt 1 (Right)', 'FontSize', title_size);
legend({'Measurement', 'EKF Prediction'}, ...
        'FontSize', legend_size, 'Location', 'best');

% --- Belt 2 ---
subplot(2,1,2); hold on; grid on;
plot(time, z_meas_store(2,:), 'b', 'LineWidth', 1.5);      % Belt 2 (Sensor)
plot(time, y_hat_store(2,:), 'g', 'LineWidth', 1.5);       % Belt 2 (EKF)
set(gca, 'FontSize', axis_size);
xlim([0 330]);
ylabel('Velocity [m/s]', 'FontSize', label_size);
xlabel('Time (s)', 'FontSize', label_size);
title('Belt 2 (Left)', 'FontSize', title_size);
legend({'Measurement', 'EKF Prediction'}, ...
        'FontSize', legend_size, 'Location', 'best');

% Barometer
figure('Name','Depth: Sensor vs EKF','NumberTitle','off');
hold on; grid on;
plot(time, z_meas_store(9,:), 'b', 'LineWidth', 1.5);   % Depth (Sensor)
plot(time, y_hat_store(9,:),  'g', 'LineWidth', 1.5);   % Depth (EKF)
set(gca, 'FontSize', axis_size);
xlim([0 330]);
ylabel('Depth [m]', 'FontSize', label_size);
xlabel('Time (s)', 'FontSize', label_size);
title('Barometer (Depth Sensor)', 'FontSize', title_size);
legend({'Measurement', 'EKF Prediction'}, ...
        'FontSize', legend_size, 'Location', 'best');
set(gca, 'YDir','reverse');

% Magnetometers
figure('Name','Magnetometer Axes: Sensor vs EKF','NumberTitle','off');

mag_names = {'x - axis', 'y - axis', 'z - axis'};
mag_colors_sensor = {'b', 'b', 'b'};
mag_colors_ekf    = {'g', 'g', 'g'};

for i = 1:3
    % Mag1 axis i
    subplot(3,2,2*i-1); hold on; grid on;
    plot(time(1:end-1), z_meas_store(2+i,1:end-1), mag_colors_sensor{i}, 'LineWidth', 1.5);  % Mag1 axis (Sensor)
    plot(time(1:end-1), y_hat_store(2+i,1:end-1),  mag_colors_ekf{i},    'LineWidth', 1.5);  % Mag1 axis (EKF)
    set(gca, 'FontSize', axis_size);
    xlim([0 330]);
    ylabel('Normalized', 'FontSize', label_size);
    if i==3, xlabel('Time (s)', 'FontSize', label_size); end
    title(['Mag1: ' mag_names{i}], 'FontSize', title_size);
    legend({'Measurement', 'EKF Prediction'}, ...
        'FontSize', legend_size-2, 'Location', 'best');

    % Mag2 axis i
    subplot(3,2,2*i); hold on; grid on;
    plot(time(1:end-1), z_meas_store(5+i,1:end-1), mag_colors_sensor{i}, 'LineWidth', 1.5);  % Mag2 axis (Sensor)
    plot(time(1:end-1), y_hat_store(5+i,1:end-1),  mag_colors_ekf{i},    'LineWidth', 1.5);  % Mag2 axis (EKF)
    set(gca, 'FontSize', axis_size);
    xlim([0 330]);
    if i==3, xlabel('Time (s)', 'FontSize', label_size); end
    title(['Mag2: ' mag_names{i}], 'FontSize', title_size);
    legend({'Measurement', 'EKF Prediction'}, ...
        'FontSize', legend_size-2, 'Location', 'best');
end

%% ========== Figure 4: EKF States (Separate Plots) ==========
for j = 1:5
    figure('Name', ['EKF State: ' state_titles{j}], 'NumberTitle','off');
    plot(time, x_hat_pred_store(j,:), 'b:', 'LineWidth', 1.2); hold on;
    plot(time, x_hat_store(j,:),     'r-', 'LineWidth', 1.5);
    ylabel(state_titles{j}, 'FontSize', label_size);
    xlabel('Time (s)', 'FontSize', label_size);
    title(['EKF State: ' state_titles{j}], 'FontSize', title_size);
    legend({'EKF Prediction', 'EKF Estimate'}, 'FontSize', legend_size, 'Location', 'best');
    set(gca, 'FontSize', axis_size);
    xlim([0 330]);
    grid on;
end


%% ===================== EKF Functions =====================
function x_next = transition_model(states, input, dt, sim_params, r_net)
    tmp = syst_dynamics(states, input, sim_params, r_net);
    x_next = states + dt * tmp;
end

function x_hat_dot = syst_dynamics(states, input, sim_params, r_net) 
    % --- Params
    m = sim_params(1); 
    Xu = sim_params(2); Xuu = sim_params(3); Xu_dot = sim_params(4);
    I = sim_params(5); 
    Nr = sim_params(6); Nrr = sim_params(7); Nr_dot = sim_params(8);
    
    psi = states(3);
    u = states(4);
    r = states(5);

    tau_u = input(1);
    tau_r = input(2);
    
    % --- Calculations
    alpha_dot = u * sin(-psi) / r_net;
    D_dot =     u * cos(-psi);
    psi_dot =   r;
    u_dot = (tau_u - Xu * u - Xuu * abs(u) * u) / (m - Xu_dot); % (Eq. 25)
    r_dot = (tau_r - Nr * r - Nrr * abs(r) * r) / (I - Nr_dot); % (Eq. 26)

    x_hat_dot = [alpha_dot;
                 D_dot;
                 psi_dot;
                 u_dot;
                 r_dot];
end

function z = measurement_model(states, theta, T)
    alpha = states(1); D = states(2); psi = states(3);
    u = states(4); r = states(5);

    belt_speeds = T \ [u; r];

    mag1 = [-cos(alpha)*cos(theta); 
             cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta); 
             cos(psi)*sin(theta) + sin(alpha)*cos(theta)*sin(psi)];
    mag2 = [-cos(alpha)*cos(theta); 
             cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta); 
             cos(psi)*sin(theta) + sin(alpha)*cos(theta)*sin(psi)];

    baro = D;

    z = [belt_speeds; mag1; mag2; baro];
end

function A = state_jacobian(states, dt, r_net, sim_params)
    % --- Params
    m = sim_params(1); 
    Xu = sim_params(2); Xuu = sim_params(3); Xu_dot = sim_params(4);
    I = sim_params(5); 
    Nr = sim_params(6); Nrr = sim_params(7); Nr_dot = sim_params(8);

    psi = states(3);
    u = states(4);
    r = states(5);

    % Jacobian of the motion model w.r.t. state
    A = eye(5);

    A(1,3) = dt * u * (cos(-psi)/r_net);
    A(1,4) = dt * (sin(-psi)/r_net);

    A(2,3) = -dt * u * sin(-psi);
    A(2,4) = dt * cos(-psi);

    A(3,5) = dt;

    A(4,4) = 1 - (dt * ((Xu + 2*Xuu*abs(u)) / (m-Xu_dot)));
    
    A(5,5) = 1 - (dt * ((Nr + 2*Nrr*abs(r)) / (I-Nr_dot)));
end

function B = input_jacobian(sim_params, dt)
    % --- Params
    m = sim_params(1); Xu_dot = sim_params(4);
    I = sim_params(5); Nr_dot = sim_params(8);
    
    % Jacobian of the motion model w.r.t. state
    B = zeros(5, 2);
    B(4,1) = dt / (m - Xu_dot);
    B(5,2) = dt / (I - Nr_dot);
end

function H = measurement_jacobian(states, theta, T)
    alpha = states(1);
    psi   = states(3);

    H = zeros(6, 5);

    % 1–2: Encoder → derivative w.r.t. [u; r]
    H(1:2,4:5) = inv(T);

    % 3–5: Magnetometer 1 rows
    H(3, :) = [sin(alpha)*cos(theta), 0, 0, 0, 0];
    H(4, :) = [cos(alpha)*cos(psi)*cos(theta), 0, - cos(psi)*sin(theta) - sin(alpha)*cos(theta)*sin(psi), 0, 0];
    H(5, :) = [cos(alpha)*cos(theta)*sin(psi), 0, cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta), 0, 0];
    % 6–8: Magnetometer 2 rows
    H(6, :) = [sin(alpha)*cos(theta), 0, 0, 0, 0];
    H(7, :) = [cos(alpha)*cos(psi)*cos(theta), 0, - cos(psi)*sin(theta) - sin(alpha)*cos(theta)*sin(psi), 0, 0];
    H(8, :) = [cos(alpha)*cos(theta)*sin(psi), 0, cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta), 0, 0];

    % 6: Barometer → depth directly
    H(9, 2) = 1;
end

function [x_hat_corr, P_corr] = EKF_correction(x_hat_pred, P_pred, z_meas, R, H, theta, T)
    y_tilde = z_meas - measurement_model(x_hat_pred, theta, T);
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    x_hat_corr = x_hat_pred + K * y_tilde;
    P_corr = (eye(size(P_pred)) - K * H) * P_pred;
end