close all;

%% Figure 1: Distribution between time samples
figure;
time_vars = {'time_imu', 'time_rpm', 'time_mag2', 'time_pos'};
titles = {'MAG1', 'RPM', 'MAG2', 'BAROMETER'};

for k = 1:length(time_vars)
    t = evalin('base', time_vars{k});
    dt = t(2:end) - t(1:end-1);

    subplot(2,2,k);
    plot(t(1:end-1), dt, '.-');
    grid on;
    set(gca, 'FontSize', axis_size);
    title([titles{k} ' time distribution between samples'], 'FontSize', title_size);
    xlabel('Time [s]', 'FontSize', label_size);
    ylabel('\Deltat [s]', 'FontSize', label_size);
end

%% Figure 2: Comparing Wheel Distance and RPM calc.
figure;
hold on;
plot(time_wheel, belt_1_speed_wheel, '-', 'LineWidth', 1, 'Color', [0.3010 0.7450 0.9330]);
plot(time_wheel, belt_2_speed_wheel, '-', 'LineWidth', 1, 'Color', [0.8500 0.3250 0.0980]);
plot(time_rpm, belt_1_speed, '-', 'LineWidth', 1, 'Color', [0 0.4 0.8]);
plot(time_rpm, belt_2_speed, '-', 'LineWidth', 1, 'Color', [0.8 0 0]);
set(gca, 'FontSize', axis_size);
hold off;
grid on;

legend({'Belt 1 speed from wheel dist', 'Belt 2 speed from wheel dist', ...
        'Belt 1 speed from rpm', 'Belt 2 speed from rpm'}, 'FontSize', legend_size);

xlabel('Time [s]', 'FontSize', label_size);
ylabel('Speed [m/s]', 'FontSize', label_size);
title('Comparing Wheel Distance vs. RPM calculations for belt speeds', 'FontSize', title_size);

%% Figure 3: Normalized mag data plot
figure;
% X axis
subplot(3,1,1);
hold on;
plot(time_imu, mag1_x, '-', 'Color', 'r', 'LineWidth', 1);
plot(time_mag2, mag2_x, '-', 'Color', 'b', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);

% xlabel('Time [s]', 'FontSize', label_size);
xlim([0 330]);
ylabel('Normalized', 'FontSize', label_size);
legend({'mag1_x', 'mag2_x'}, 'FontSize', legend_size);
title('Magnetometer Data', 'FontSize', title_size);
hold off;
grid on;

% Y axis
subplot(3,1,2);
hold on;
plot(time_imu, mag1_y, '-', 'Color', 'r', 'LineWidth', 1);
plot(time_mag2, mag2_y, '-', 'Color', 'b', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);

% xlabel('Time [s]', 'FontSize', label_size);
xlim([0 330]);
ylabel('Normalized', 'FontSize', label_size);
legend({'mag1_y', 'mag2_y'}, 'FontSize', legend_size);
% title('Magnetometer Y (normalized)', 'FontSize', title_size);
hold off;
grid on;

% Z axis
subplot(3,1,3);
hold on;
plot(time_imu, mag1_z, '-', 'Color', 'r', 'LineWidth', 1);
plot(time_mag2, mag2_z, '-', 'Color', 'b', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);

xlabel('Time [s]', 'FontSize', label_size);
xlim([0 330]);
ylabel('Normalized', 'FontSize', label_size);
legend({'mag1_z', 'mag2_z'}, 'FontSize', legend_size);
% title('Magnetometer Z (normalized)', 'FontSize', title_size);
hold off;
grid on;

%% Figure 4: Acceleration data plot
figure;

% X axis
subplot(3,1,1);
plot(time_imu, acc_x, '-', 'Color', 'r', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);
% xlabel('Time [s]', 'FontSize', label_size);
xlim([0 330]);
ylabel('[m/s^2]', 'FontSize', label_size);
legend({'acc_x'}, 'FontSize', legend_size);
title('Acellerometer Data', 'FontSize', title_size);


hold off;
grid on;

% Y axis
subplot(3,1,2);
plot(time_imu, acc_y, '-', 'Color', 'r', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);
% xlabel('Time [s]', 'FontSize', label_size);
xlim([0 330]);
ylabel('[m/s^2]', 'FontSize', label_size);
legend({'acc_y'}, 'FontSize', legend_size);
% title('Acc y', 'FontSize', title_size);

hold off;
grid on;

% Z axis
subplot(3,1,3);
plot(time_imu, acc_z, '-', 'Color', 'r', 'LineWidth', 1);
set(gca, 'FontSize', axis_size);
xlabel('Time [s]', 'FontSize', label_size);
xlim([0 330]);
ylabel('[m/s^2]', 'FontSize', label_size);
legend({'acc_z'}, 'FontSize', legend_size);
% title('Acc z', 'FontSize', title_size);

hold off;
grid on;
