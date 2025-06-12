%% Underwater Biofouling Prevention Robot Simulation with EKF
% This script sets up a simulation environment for an underwater net 
% grooming robot, produced by Remora Robotics.


%% ===================== Changeable Params for testing =====================
clear;               % Clears all params in Workspace
% close all;         % Optional: Uncomment to close all existing figures before running.

% ---- Latitude and Longitude of test location ----
lat = 63.41;
lon = 10.41;

% ---- Net Radius ----
radius_net = 25.5;   % Radius of the net structure (m)

% ---- Current Parameters ----
V_current = 0.0;     % Water current influencing the movement of the robot (m/s)
beta_current = 0;    % Current direction in rad

% ---- Covariance matrices (Tuning parameters for the EKF) ----
% Initial error covariance
P = eye(5) * 0.1; 
% Process noise covariance matrix Q that reflects model uncertainties
Q = eye(5) * 0.01;
% Measurement noise covariance matrix R.
R = eye(6) * 0.05;

% ---- Reference input signals for the Simulator ----
switchTimes  = [50,   110,  135,   192];
switchValues = [0, (0 + pi/2), (pi/2 + pi/2), (pi + pi/2), 2*pi];

u_ref = @(t) (t < 2) * 0 + ...
             (t >= 2 & t < switchTimes(4)) * 0.3 + ...
             (t >= switchTimes(4)) * -0.3;

psi_ref = @(t) (t <  switchTimes(1))                      * switchValues(1) + ...
               (t >= switchTimes(1) & t < switchTimes(2)) * switchValues(2) + ...
               (t >= switchTimes(2) & t < switchTimes(3)) * switchValues(3) + ...
               (t >= switchTimes(3) & t < switchTimes(4)) * switchValues(4) + ...
               (t >= switchTimes(4))                      * switchValues(5);

% For changing the dynamic net introduction parameters, go to line 145.


%% ===================== Time Params =====================
time_step = 0.01; dt = time_step;
sim_time = 218;
num_steps = sim_time / time_step;


%% ===================== Physical params for net and robot =====================
r_net = radius_net;     % Radius of net
l = 0.13;               % Moment arm (m)
T = [1/2,     1/2; 
    -1/(2*l), 1/(2*l)]; % Encoder m/s to u and r translation

date_test = [30, 05, 2025]; % Day, Month, Year
% NTNU Gløs: lon = 63.41, lat = 10.41
% Remora Robotics Office: lon = 58.98, lat = 5.71
lat_test = lat;
lon_test = lon;
[XYZ, hor_intensity, declination, inclination, tot_intensity] = ... 
    wrldmagm(0, lat_test, lon_test, decyear(date_test(3), date_test(2), date_test(1)),'Custom','WMM.COF');
theta = deg2rad(inclination);


%% ===================== Surge and heading controllers init =====================
mass = 24;      % Mass (kg)
inertia = 5;    % Inertia around z-axis (kg*m^2)
Xu_dot = -2*2;  % Added mass in surge
Nr_dot = -1;    % Added inertia in yaw
Xu = -3*2;      % Linear damping in surge
Nr = -1;        % Linear damping in yaw
Xuu = -1*2;     % Nonlinear damping in surge
Nrr = -0.1;     % Nonlinear damping in yaw

% Test: Ideal Conditions
sim_params = [mass, Xu, Xuu, Xu_dot, inertia, Nr, Nrr, Nr_dot]; 

% Test: Ultimate
est_params = sim_params + randn(size(sim_params)) * 0.5; 


%% ===================== Simulator params and init =====================
% Initial Conditions
alpha = 0;      % (rad)
D     = 0;      % (m)
psi   = 0;      % (rad)
u     = 0;      % (m/s)
r     = 0;      % (rad/s)

% Initialize States
states = zeros(5, num_steps);
states(:, 1) = [alpha, D, psi, u, r];

% Initialize Integral Terms
u_integral = 0;
psi_integral = 0;

% Control Gains
k1 = 4; k2 = 4;         % Gains for surge controller
k3 = 5; k4 = 5; k5 = 1; % Gains for heading controller

% ------- For second and third order ref generation -------
omega1 = 1; % Natural frequency for surge control
omega2 = 2; % Natural frequency for heading control
zeta = 1;   % Damping ratio

u_d = 0; u_d_dot = 0; u_d_ddot = 0;
psi_d = 0; psi_d_dot = 0; psi_d_ddot = 0; psi_d_3dot = 0;

% ------- Cylindrical Coordinate Transformation (From NED to cylindrical) -------
J_c = @(psi, radius_net) [sin(-psi)/radius_net, 0; 
                          cos(-psi),            0; 
                          0,                    1];


%% ===================== Current introduction parasm and init =====================
v_c = V_current;       % Water current influencing the movement of the robot (m/s)
beta_c = beta_current; % Current direction in rad
n_z = [0, 0, 1];       % Normal vector in z-direction for current contribution

% Initialize Current vectors
V_c_c = zeros(num_steps, 3);
% V_c_body = zeros(num_steps, 3);
V_c_r = zeros(num_steps, 3);

% Storage for simulation results
u_c_r = zeros(num_steps, 1);
u_r = zeros(num_steps, 1);
u_d_store = zeros(1, num_steps);

u_tilde_store = zeros(1, num_steps);
psi_tilde_store = zeros(1, num_steps);


%% ===================== Dynamic net introduction init and params =====================
r_net_t = zeros(1, num_steps);
% Dynamic net parameters (tune these as needed)
r_net_min = r_net - 1;    % Minimum allowable net radius (m)
r_net_max = r_net + 1;    % Maximum allowable net radius (m)

if v_c == 0
    amp = 0.1; % Test: Ideal Conditions
else
    amp = 1.5 * v_c; % Test: Gradually Harsher and Ultimate
end

% First-order net dynamics gain
k_net = 1; % Adjust to tune how quickly the net radius transitions to 
           % the new random target

% Time between random radius setpoint changes (in seconds)
refresh_period = 30.0;

% Wave parameters for periodic variations
T_wave = 30;  % Wave period in seconds (adjust as needed)
phase = 0;    % Phase offset (if needed)

% Compute average and amplitude
r_net_avg = (r_net_min + r_net_max) / 2;
r_net_amp = amp * (r_net_max - r_net_min) / 2;

% Initialize net radius state
r_net_state = r_net;
r_net_state_store = zeros(1, num_steps);
r_net_state_store(1) = r_net_state;
r_net_desired = r_net_state;


%% ===================== EKF init and params =====================
input = zeros(2, num_steps); % [tau u, tau r from Eq. 33 and 36]

% Initial state estimate
x_hat = states(:, 1);  % Test: Perfect Conditions

% Test: Ultimate harsh Conditions
% x_hat = states(:, 1) + [ ...
%     randn(1) * 0.5;   % For alpha
%     randn(1) * 5;     % For D
%     randn(1) * 0.5;   % For psi
%     randn(1) * 0.1;   % For u
%     randn(1) * 0.1];  % For r 

x_hat_store = zeros(5, num_steps);
x_hat_store(:,1) = x_hat;

y_hat_times_H_store = zeros(6, num_steps);
y_hat_store         = zeros(6, num_steps);
z_meas_store        = zeros(6, num_steps);


%% ===================== Simulator loop =====================
for i = 1:num_steps-1
    t = (i-1) * time_step;

    %% -------- Extract States --------
    alpha = states(1, i);
    D     = states(2, i);
    psi   = states(3, i);
    u     = states(4, i);
    r     = states(5, i);

    %% -------- Current contribution: Attempt 1 (SINTEF) --------
    V_c_c(i, :) = [v_c*cos(beta_c - alpha), v_c*sin(beta_c - alpha), 0];
    V_c_r(i, :) = cross(V_c_c(i, :), n_z);
    u_c_r_vec = V_c_r(i, 1);
    u_c_r(i) = u_c_r_vec * sin(psi);
    u_r(i) = u - u_c_r(i);
    ur = u_r(i);

    %% -------- Control Laws --------
    % Reference Signals
    u_ref_val = u_ref(t);
    psi_ref_val = wrapTo2Pi(psi_ref(t));

    % Reference Model for Surge Speed
    u_d_ddot = omega1^2 * (u_ref_val - u_d) - 2 * zeta * omega1 * u_d_dot;
    u_d_dot = u_d_dot + u_d_ddot * time_step;
    u_d = u_d + u_d_dot * time_step;
    u_d_store(i) = u_d;

    % Reference Model for Heading Angle and Speed
    psi_d_3dot = omega2^3 * (psi_ref_val - psi_d) - (2*zeta + 1) * omega2^2 * psi_d_ddot - (2*zeta + 1) * omega2^2 * psi_d_dot;
    psi_d_ddot = psi_d_ddot + psi_d_3dot * time_step;
    psi_d_dot = psi_d_dot + psi_d_ddot * time_step;
    psi_d = psi_d + psi_d_dot * time_step;
    r_d_dot = psi_d_ddot;
    r_d = psi_d_dot;

    % Calculating errors for psi, u and r
    psi_tilde = psi - psi_d;
    u_tilde = ur - u_d;
    r_tilde = r - r_d;
    u_tilde_store(i) = u_tilde;
    psi_tilde_store(i) = psi_tilde;

    % Surge Speed Control (Eq. 33)
    tau_u = Xu*ur + Xuu*abs(ur)*ur + (mass - Xu_dot)*(u_d_dot - k1*u_tilde - k2*u_integral);
    input(1, i) = tau_u;
    % Heading Control (Eq. 36)
    tau_r = Nr*r + Nrr*abs(r)*r - (inertia - Nr_dot)*(k3*psi_tilde + k4*r_tilde  + k5*psi_integral + r_d_dot);    
    input(2, i) = tau_r;

    %% -------- Update Net Radius Dynamically --------
    % --- Attempt 3 (Wave Structuer: Update the net radius using a sinusoidal function)
    r_net_state = r_net_avg + r_net_amp * sin(2*pi/T_wave * t + phase);
    r_net_state_store(i+1) = r_net_state;
    
    %% -------- Simulator update --------
    u_integral = u_integral + u_tilde * time_step;
    psi_integral = psi_integral + psi_tilde * time_step;
    
    x_hat_dot = syst_dynamics(states(:, i), input(:, i), sim_params, ur, r_net_state);
    u_dot = x_hat_dot(4);
    r_dot = x_hat_dot(5);
    eta_dot = J_c(psi, r_net_state) * [ur; r];

    %% -------- EKF --------
    % EKF Prediction 
    % Test: Ideal and Harsh Conditions
    A = state_jacobian(x_hat, time_step, r_net_state, sim_params);
    B = input_jacobian(sim_params, dt);

    x_hat_pred = transition_model(x_hat, input(:, i), dt, sim_params, ur, r_net_state);
    
    % Test: Ultimate
    % A = state_jacobian(x_hat, time_step, r_net_state, est_params);
    % B = input_jacobian(est_params, dt);
    % 
    % x_hat_pred = transition_model(x_hat, input(:, i), dt, est_params, ur, r_net_state);
    
    P_pred = A * P * A' + Q;
    
    % Measurement Update (EKF Correction)
    z_meas = measurement_model(states(:, i), theta, T);
    z_meas_store(:, i) = z_meas;

    H = measurement_jacobian(x_hat_pred, theta, T);
    [x_hat, P] = EKF_correction(x_hat_pred, P_pred, z_meas, R, H, theta, T);
    
    % Store EKF estimates
    x_hat_store(:, i+1) = x_hat;
    y_hat_times_H_store(:, i) = H * x_hat;
    y_hat_store(:, i) = measurement_model(x_hat_pred, theta, T);

    %% -------- States update --------
    states(:, i+1) = states(:, i) + time_step * [eta_dot(1); eta_dot(2); r; u_dot; r_dot];
end


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

%% ===================== Functions =====================
% -------------------- EKF --------------------
function x_next = transition_model(states, input, dt, sim_params, ur, r_net_dyn)
    x_hat = states;
    x_next = x_hat + dt * syst_dynamics(x_hat, input, sim_params, ur, r_net_dyn);
end

function x_hat_dot = syst_dynamics(states, input, sim_params, ur, r_net_dyn)
    % --- Params
    m = sim_params(1); 
    Xu = sim_params(2); Xuu = sim_params(3); Xu_dot = sim_params(4);
    I = sim_params(5); 
    Nr = sim_params(6); Nrr = sim_params(7); Nr_dot = sim_params(8);
    r_net = r_net_dyn;
    
    psi = states(3); r = states(5);

    tau_u = input(1);
    tau_r = input(2);
    
    % --- Calculations
    alpha_dot = ur * sin(-psi) / r_net;
    D_dot =     ur * cos(-psi);
    psi_dot =   r;
    u_dot = (tau_u - Xu * ur - Xuu * abs(ur) * ur) / (m - Xu_dot);
    r_dot = (tau_r - Nr * r - Nrr * abs(r) * r) / (I - Nr_dot);

    x_hat_dot = [alpha_dot;
                 D_dot;
                 psi_dot;
                 u_dot;
                 r_dot];
end

function [x_hat_corr, P_corr] = EKF_correction(x_hat_pred, P_pred, z_meas, R, H, theta, T)
    y_tilde = z_meas - measurement_model(x_hat_pred, theta, T);
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    x_hat_corr = x_hat_pred + K * y_tilde;
    P_corr = (eye(size(P_pred)) - K * H) * P_pred;
end

function z = measurement_model(states, theta, T)
    alpha = states(1);
    D = states(2);
    psi = states(3);
    u = states(4);
    r = states(5);

    % ----- Sensor Noise -----
    encoder_variance = 0.15; % m/s
    encoder_noise = encoder_variance^2 * randn(2,1);
    
    mag_variance = 50 / 1000; % mG
    mag_noise = mag_variance^2 * randn(3,1);    
    % Magnetometer 1: MMC5983MA
        % It can measure magnetic fields within the full scale range of +/- 8 Gauss (G), 
        % with 0.25 mG/0.0625 mG per LSB resolution at 16bits/18bits 
        % operation mode and 0.4 mG total RMS noise level, 
        % enabling heading accuracy of +/- 0.5º in electronic compass applications.
        % With 16 bits operation: 4096 Counts/G = LSB
        % With 18 bits operation: 16 384 Counts/G = LSB
    % Magnetometer 2: LIS3MDL
         % Measurement range +/- 4 gaus: 6842 LSB/ gauss
         % Measurement range +/- 8 gaus: 3421 LSB/ gauss
         % Measurement range +/- 12 gaus: 2281 LSB/ gauss
             % X-axis RMS noise: 3.2 * 10^(-3) gauss
             % Y-axis RMS noise: 3.2 * 10^(-3) gauss
             % Z-axis RMS noise: 4.1 * 10^(-3) gauss
         % Measurement range +/- 16 gaus: 1711 LSB/ gauss

    baro_variance = 0.1; % m
    baro_noise = baro_variance^2 * randn(1);
        % With a pressure resolution of 3 mbar, it provides depth readings 
        % with a precision of approximately 3 cm.

    % ----- Encoder / belt speeds -----
    belt_speeds = T \ [u; r] + encoder_noise;

    % ----- Magnetometer 1 -----
    mag = [-cos(alpha)*cos(theta); 
             cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta); 
             cos(psi)*sin(theta) + sin(alpha)*cos(theta)*sin(psi) ] ...
           + mag_noise;

    % ----- Barometer -----
    baro = D + baro_noise;

    % ----- Measurement vector (6×1) -----
    z = [ belt_speeds;
          mag;
          baro ];
end

% -------------------- Jacobians --------------------
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

    % 3–5: Magnetometer derivative
    H(3, :) = [sin(alpha)*cos(theta), 0, 0, 0, 0];
    H(4, :) = [cos(alpha)*cos(psi)*cos(theta), 0, - cos(psi)*sin(theta) - sin(alpha)*cos(theta)*sin(psi), 0, 0];
    H(5, :) = [cos(alpha)*cos(theta)*sin(psi), 0, cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta), 0, 0];

    % 6: Barometer derivative
    H(6, 2) = 1;
end


% -------------------- Plotting --------------------
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