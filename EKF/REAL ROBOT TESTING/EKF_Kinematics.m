close all;

%% Params
num_steps = length(time);
dt = 0.1;

r_net = 25.5;
l = 0.13;

lat_test = 58.221967; % THESE NEED TO BE CHANGED FOR EACH TEST
lon_test = 6.68495;   % THESE NEED TO BE CHANGED FOR EACH TEST
[XYZ, hor_intensity, declination, inclination, tot_intensity] = ... 
    wrldmagm(0, lat_test, lon_test, decyear(2025,5,8),'Custom','WMM.COF');

theta = deg2rad(inclination);
% theta = 0.47; For debugging based on the results from the changing theta test
T_encoder_speed = [1/2,  1/2; 1/(2*l), -1/(2*l)];

%% -------------------- EKF loop Init --------------------
% Define initial values if not defined earlier:
x_hat = [0; 0; 0];
x_hat_store = zeros(3, num_steps);
x_hat_store(:, 1) = x_hat;

% ------- Covariance matrices (Tuning parameters for the EKF) -------
P = eye(3) * 0.1;  % Initial covariance
    % How sure you are on the initial guess (x_hat)!

% Process noise covariance matrix Q that reflects model uncertainties
    % Low number = Thrust!

% Measurement noise covariance matrix R.
    % Low number = Thrust!
    % If 0. Sensor data is 100% correct. Throw away the entire model!

% It is the relationship between these that is important!

%% Test 1: Trying to better alpha estimates by trusting the math model
    % Answer: Quite good!
% lower Q(1,1) % Trust the mathematical model 
Q = diag([0.0001, ... % Alpha
          0.01, ...   % D
          0.01]);    % Psi
% increase R(1,1), R(4,4) % Don't trust the measurments og mag x!
R = diag([0.5, ...    % Mag1 x
          0.01, ...   % Mag1 y
          0.02, ...   % Mag1 z
          0.5, ...    % Mag2 x
          0.01, ...   % Mag2 y
          0.02, ...   % Mag2 z
          0.005]);    % Barometer

%% Test 2: Testing measurments reliability
    % Answer: Not Good!
% % High Q(1,1) % Dont trust math model. Want to check measurments!
% Q = eye(3) * 1; % Sensor measurment test
% % dECREASE R(1,1), R(4,4) % TRUST MEASURMNETS
% R = eye(7) * 0.01;
                             
z_meas_store = zeros(7, num_steps);
y_hat_store  = zeros(7, num_steps);
x_hat_pred_store = zeros(3, num_steps);
states_store = zeros(3, num_steps);
input = zeros(2, num_steps);

%% -------------------- EKF loop --------------------
for i = 1:num_steps-1
    % --- EKF prediction step ---
    belt_speeds_meas = [belt_1_speed_wheel(i); belt_2_speed_wheel(i)];
    input(:, i)      = T_encoder_speed * belt_speeds_meas;

    A = state_jacobian(x_hat, input(:, i), dt, r_net);
    B = input_jacobian(x_hat, dt, r_net);
    P_pred = A * P * A' + Q;
    x_hat_pred = transition_model(x_hat, input(:, i), dt, r_net);
    x_hat_pred_store(:, i) = x_hat_pred;

    y_hat_store(:, i) = measurement_model(x_hat_pred, theta);

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

    y_tilde = z_meas - measurement_model(x_hat_pred, theta);

    % Compute H for the current x_hat_pred
    H = measurement_jacobian(x_hat_pred, theta);

    [x_hat, P] = EKF_correction(x_hat_pred, P_pred, z_meas, R, H, theta);

    x_hat_store(:, i+1) = x_hat;
end

%% --- Titles for each state
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
% belt1_speed_est = zeros(1,num_steps);
% belt2_speed_est = zeros(1,num_steps);
% for i = 1:num_steps
%     belts2 = input(:,i); % input = [u; r]
%     belt_speeds    = T_encoder_speed \ belts2;
%     belt1_speed_est(i) = belt_speeds(1);
%     belt2_speed_est(i) = belt_speeds(2);
% end
% 
% figure('Name','Figure 6: Estimated right and left belt speeds','NumberTitle','off');
% subplot(2,1,1); hold on; grid on;
% plot(time(1:end-1), belt1_speed_est(1:end-1), 'r', 'LineWidth', 1.5);
% ylabel('Belt Right (m/s)', 'FontSize', label_size);
% title('Right Belt speed', 'FontSize', title_size);
% set(gca, 'FontSize', axis_size);
% 
% subplot(2,1,2); hold on; grid on;
% plot(time(1:end-1), belt2_speed_est(1:end-1), 'r', 'LineWidth', 1.5);
% ylabel('Belt Left (m/s)', 'FontSize', label_size);
% xlabel('Time (s)', 'FontSize', label_size);
% title('Left Belt Speed', 'FontSize', title_size);
% set(gca, 'FontSize', axis_size);


%% Functions
function x_next = transition_model(states, input, dt, r_net)
    x_hat_dot = syst_dynamics(states, input, r_net);
    x_next = states + dt * x_hat_dot;
end

function x_hat_dot = syst_dynamics(states, input, r_net) 
    psi = states(3);

    u = input(1);
    r = input(2);
    
    % --- Calculations
    alpha_dot = u * sin(-psi) / r_net;
    D_dot =     u * cos(-psi);
    psi_dot =   r;

    x_hat_dot = [alpha_dot;
                 D_dot;
                 psi_dot];
end

function z = measurement_model(states, theta)
    alpha = states(1); D = states(2); psi = states(3);
    % u = states(4); r = states(5);

    % belt_speeds = T \ [u; r];

    % u_dot og r_dot kan vel skrotes for å bruke mer politelig data!?
    % rpm. Se på kalkulasjoner fra Håkon!

    mag1 = [-cos(alpha)*cos(theta); 
             cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta); 
             cos(psi)*sin(theta) + sin(alpha)*cos(theta)*sin(psi)];
    mag2 = [-cos(alpha)*cos(theta); 
             cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta); 
             cos(psi)*sin(theta) + sin(alpha)*cos(theta)*sin(psi)];

    baro = D;

    z = [mag1; mag2; baro];
end


%% -------------------- Jacobian Functions --------------------
function A = state_jacobian(states, input, dt, r_net)
    psi = states(3);

    u = input(1);

    % Jacobian of the motion model w.r.t. state
    A = eye(3);

    A(1,3) = dt * u * (cos(-psi)/r_net);
    A(2,3) = -dt * u * sin(-psi);
end

function B = input_jacobian(states, dt, r_net)
    psi = states(3);
    % Jacobian of the motion model w.r.t. state
    B = dt * [sin(-psi)/r_net, 0; 
              cos(-psi),            0; 
              0,                    1];
end

function H = measurement_jacobian(states, theta)
    alpha = states(1);
    psi   = states(3);

    % Start with zeros
    H = zeros(7, 3);

    % 3–5: Magnetometer 1 rows
    H(1, :) = [sin(alpha)*cos(theta), 0, 0];
    H(2, :) = [cos(alpha)*cos(psi)*cos(theta), 0, - cos(psi)*sin(theta) - sin(alpha)*cos(theta)*sin(psi)];
    H(3, :) = [cos(alpha)*cos(theta)*sin(psi), 0, cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta)];
    % 6–8: Magnetometer 2 rows
    H(4, :) = [sin(alpha)*cos(theta), 0, 0];
    H(5, :) = [cos(alpha)*cos(psi)*cos(theta), 0, - cos(psi)*sin(theta) - sin(alpha)*cos(theta)*sin(psi)];
    H(6, :) = [cos(alpha)*cos(theta)*sin(psi), 0, cos(psi)*sin(alpha)*cos(theta) - sin(psi)*sin(theta)];

    % 6: Barometer → depth directly
    H(7, 2) = 1;
end


%% -------------------- EKF Function --------------------
function [x_hat_corr, P_corr] = EKF_correction(x_hat_pred, P_pred, z_meas, R, H, theta)
    y_tilde = z_meas - measurement_model(x_hat_pred, theta);
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
        % NB: Finnes en alt. metode uten matriseinvertering!!
        % Gjør kalkulasjonene lettere for datamaskinen
    x_hat_corr = x_hat_pred + K * y_tilde;
    P_corr = (eye(size(P_pred)) - K * H) * P_pred;
end