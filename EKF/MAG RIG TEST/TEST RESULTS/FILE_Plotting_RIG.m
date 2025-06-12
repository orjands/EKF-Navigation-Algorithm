%% Additional Plots from Test Rig Evaluation
% Generates supplementary plots based on the test rig experiment data.

% Note: Run 'FILE_Load_RIG.m' first to ensure all necessary data is loaded.

% close all; % Optional: Uncomment to close all existing figures before running.


%% ===================== Figure 1: Distribution between time samples =====================
figure;
time_vars = {'time_imu', 'time_mag2'};
titles = {'MAG1', 'MAG2'};

for k = 1:length(time_vars)
    t = evalin('base', time_vars{k});
    dt = t(2:end) - t(1:end-1);

    subplot(1,2,k);
    plot(t(1:end-1), dt, '.-'); grid on;
    set(gca, 'FontSize', axis_size);
    title([titles{k} ' time distribution between samples'], 'FontSize', title_size);
    xlabel('Time [s]', 'FontSize', label_size);
    ylabel('\Deltat [s]', 'FontSize', label_size);
    
end

%% ===================== Figure 2: Normalized mag data plot =====================
figure;

% X axis
subplot(3,1,1); hold on;
plot(time_imu, mag1_x, '-', 'Color', 'r', 'LineWidth', 1);
plot(time_mag2, mag2_x, '-', 'Color', 'b', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);
legend({'mag1_x', 'mag2_x'}, 'FontSize', legend_size);
% xlabel('Time [s]', 'FontSize', label_size);
ylabel('Normalized', 'FontSize', label_size);
title('Magnetometer Data', 'FontSize', title_size);
grid on;

% Y axis
subplot(3,1,2); hold on;
plot(time_imu, mag1_y, '-', 'Color', 'r', 'LineWidth', 1);
plot(time_mag2, mag2_y, '-', 'Color', 'b', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);
legend({'mag1_y', 'mag2_y'}, 'FontSize', legend_size);
% xlabel('Time [s]', 'FontSize', label_size);
ylabel('Normalized', 'FontSize', label_size);
% title('Magnetometer: y-axis', 'FontSize', title_size);
grid on;

% Z axis
subplot(3,1,3); hold on;
plot(time_imu, mag1_z, '-', 'Color', 'r', 'LineWidth', 1);
plot(time_mag2, mag2_z, '-', 'Color', 'b', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);
legend({'mag1_z', 'mag2_z'}, 'FontSize', legend_size);
xlabel('Time [s]', 'FontSize', label_size);
ylabel('Normalized', 'FontSize', label_size);
% title('Magnetometer: z-axis', 'FontSize', title_size);
grid on;

%% ===================== Figure 3: Acceleration data plot =====================
figure;

% X axis
subplot(3,1,1);
plot(time_imu, acc_x, '-', 'Color', 'r', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);
% xlabel('Time [s]', 'FontSize', label_size);
ylabel('[m/s^2]', 'FontSize', label_size);
legend({'acc_x'}, 'FontSize', legend_size);
title('Accelerometer Data', 'FontSize', label_size);
grid on;

% Y axis
subplot(3,1,2);
plot(time_imu, acc_y, '-', 'Color', 'r', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);
% xlabel('Time [s]', 'FontSize', label_size);
ylabel('[m/s^2]', 'FontSize', label_size);
legend({'acc_y'}, 'FontSize', legend_size);
% title('Acc y', 'FontSize', label_size);
grid on;

% Z axis
subplot(3,1,3);
plot(time_imu, acc_z, '-', 'Color', 'r', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);
xlabel('Time [s]', 'FontSize', label_size);
ylabel('[m/s^2]', 'FontSize', label_size);
legend({'acc_z'}, 'FontSize', legend_size);
% title('Acc z', 'FontSize', label_size);
grid on;

%% ===================== Figure 4: alpha and psi estimates given mag measurments =====================
lat_test = 63.41;
lon_test = 10.41;
[XYZ, hor_intensity, declination, inclination, tot_intensity] = ... 
    wrldmagm(0, lat_test, lon_test, decyear(2025,6,1),'Custom','WMM.COF');
theta = deg2rad(inclination);

cos_alpha = -mag1_x ./ cos(theta);
cos_alpha = min(max(cos_alpha, -1), 1);   % Clamp to [-1, 1]
alpha = acos(cos_alpha);

S = sin(alpha) * cos(theta);
den = S.^2 + sin(theta).^2;
epsilon = 1e-12;
den(den < epsilon) = epsilon;  % Prevent divide by zero

sin_psi = (mag1_z .* S - mag1_y * sin(theta)) ./ den;
cos_psi = (mag1_y .* S + mag1_z * sin(theta)) ./ den;

% Clamp sin_psi and cos_psi too
sin_psi = min(max(sin_psi, -1), 1);
cos_psi = min(max(cos_psi, -1), 1);

psi = atan2(sin_psi, cos_psi);

% Remove complex or NaN results
alpha(~isreal(alpha)) = NaN;
psi(~isreal(psi)) = NaN;

figure('Name','Figure 4: Alpha and Psi from Magnetometer','NumberTitle','off');
subplot(2,1,1); hold on;
plot(time_imu, rad2deg(alpha), 'r', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('[deg]', 'FontSize', label_size);
title('\alpha estimates from mag measurements', 'FontSize', title_size);
grid on;

subplot(2,1,2); hold on;
plot(time_imu, rad2deg(psi), 'b', 'LineWidth', 1.5);
set(gca, 'FontSize', axis_size);
ylabel('[deg]', 'FontSize', label_size);
xlabel('Time [s]', 'FontSize', label_size);
title('\psi estimates from mag measurements', 'FontSize', title_size);
grid on;
