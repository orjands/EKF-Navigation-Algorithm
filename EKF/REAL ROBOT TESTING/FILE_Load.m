%% Load Real-World Test Data
% Loads all necessary data from the real-world test experiments
% for running the EKF navigation algorithm.


%% ===================== Test definition =====================
clear;

test_num = 9;

idx_start = 5700; % TEST 9
idx_end = 9000;   % TEST 9

% idx_start = 0; % MAG RIG TEST 2
% idx_end = 2350;   % MAG RIG TEST 2

load_test_data(test_num, idx_start, idx_end);


%% ===================== Plotting params =====================
label_size = 20;
axis_size = 15;
title_size = 23;
legend_size = 15;
legend_size_F3_and_4 = 12;
grayColor = [0.25 0.25 0.25];


%% ===================== Load Data Function =====================
function load_test_data(test_num, idx_window_start, idx_window_end)
    % Constants
    driverwheel_radius = 0.043;  % meters

    % Create folder name string based on test number
    folder_name = sprintf('TEST%d', test_num);

    % File list, built from the folder name and test number
    esc_file      = fullfile(folder_name, sprintf('TEST%d_esc_telemetry_1_to4.csv', test_num));
    pos_file      = fullfile(folder_name, sprintf('TEST%d_Global_Pos_Int.csv', test_num));
    imu_file      = fullfile(folder_name, sprintf('TEST%d_IMU.csv', test_num));
    mag2_file     = fullfile(folder_name, sprintf('TEST%d_MAG2.csv', test_num));
    rpm_file      = fullfile(folder_name, sprintf('TEST%d_RPM.csv', test_num));
    wheeldist_file= fullfile(folder_name, sprintf('TEST%d_WheelDist.csv', test_num));
    
    % Load files
    rpm_data      = readtable(rpm_file);
    imu_data      = readtable(imu_file);
    mag2_data     = readtable(mag2_file);
    pos_data      = readtable(pos_file);
    wheel_data    = readtable(wheeldist_file);
    esc_data      = readtable(esc_file);

    % Lengths for windowing
    n_rpm    = height(rpm_data);
    n_imu    = height(imu_data);
    n_mag2   = height(mag2_data);
    n_pos    = height(pos_data);
    n_wheel  = height(wheel_data);

    if nargin < 2, idx_window_start = 1; end
    if nargin < 3
        idx_window_end = min([n_rpm, n_imu, n_mag2, n_pos, n_wheel]);
    end

    % Clamp end index for each signal in case their lengths differ slightly
    idx_rpm   = idx_window_start : min(idx_window_end, n_rpm);
    idx_imu   = idx_window_start : min(idx_window_end, n_imu);
    idx_mag2  = idx_window_start : min(idx_window_end, n_mag2);
    idx_pos   = idx_window_start : min(idx_window_end, n_pos);
    idx_wheel = idx_window_start : min(idx_window_end, n_wheel);

    % Time variables
    ts_rpm   = rpm_data.timestamp(idx_rpm);
    time_rpm = ts_rpm - ts_rpm(1);

    ts_imu   = imu_data.timestamp(idx_imu);
    time_imu = ts_imu - ts_imu(1);

    ts_mag2   = mag2_data.timestamp(idx_mag2);
    time_mag2 = ts_mag2 - ts_mag2(1);

    ts_pos   = pos_data.timestamp(idx_pos);
    time_pos = ts_pos - ts_pos(1);

    ts_wheel = wheel_data.timestamp(idx_wheel);
    time_wheel = ts_wheel - ts_wheel(1);

    ts_esc   = esc_data.timestamp(idx_rpm);
    time_esc = ts_esc - ts_esc(1);

    N = min([length(time_rpm), length(time_imu), length(time_mag2), length(time_pos), length(time_wheel), length(time_esc)]);
    time = time_imu(1:N);

    % RPM-based speed
    belt_1_rpm   = rpm_data.RPM_rpm1(idx_rpm);
    belt_2_rpm   = rpm_data.RPM_rpm2(idx_rpm);
    belt_1_speed = belt_1_rpm(:) * (2 * pi * driverwheel_radius) / (60 * 16); % [m/s]
    belt_2_speed = belt_2_rpm(:) * (2 * pi * driverwheel_radius) / (60 * 16); % [m/s]

    % Mag1 from IMU
    mag1_x = imu_data.RAW_IMU_xmag(idx_imu);
    mag1_y = imu_data.RAW_IMU_ymag(idx_imu);
    mag1_z = imu_data.RAW_IMU_zmag(idx_imu);
    norm1 = sqrt(mag1_x.^2 + mag1_y.^2 + mag1_z.^2);
    mag1_x = mag1_x ./ norm1;
    mag1_y = mag1_y ./ norm1;
    mag1_z = mag1_z ./ norm1;
    % Acc from IMU
    acc_x = imu_data.RAW_IMU_xacc(idx_imu) * (-9.81/1000);
    acc_y = imu_data.RAW_IMU_yacc(idx_imu) * (-9.81/1000);
    acc_z = imu_data.RAW_IMU_zacc(idx_imu) * (-9.81/1000);

    % Mag2 from MAG2 file
    mag2_x = mag2_data.SCALED_IMU2_xmag(idx_mag2);
    mag2_y = mag2_data.SCALED_IMU2_ymag(idx_mag2);
    mag2_z = mag2_data.SCALED_IMU2_zmag(idx_mag2);
    norm2 = sqrt(mag2_x.^2 + mag2_y.^2 + mag2_z.^2);
    mag2_x = mag2_x ./ norm2;
    mag2_y = mag2_y ./ norm2;
    mag2_z = mag2_z ./ norm2;

    % Depth measurement
    depth_meas = pos_data.GLOBAL_POSITION_INT_relative_alt(idx_pos) * (-1 / 1000);

    % Wheel distance and speed
    belt_1_dist = wheel_data.WHEEL_DISTANCE_distance(idx_wheel);
    belt_2_dist = wheel_data.Var5(idx_wheel);

    dt_wheel = [1; diff(time_wheel)];  % Avoid div by zero at first element
    belt_1_speed_wheel = [0; diff(belt_1_dist)] ./ dt_wheel;
    belt_2_speed_wheel = [0; diff(belt_2_dist)] ./ dt_wheel;

    % Voltage and Current
    voltage1 = esc_data.ESC_TELEMETRY_1_TO_4_voltage_1(idx_rpm) * 0.01;
    current1 = esc_data.ESC_TELEMETRY_1_TO_4_current_1(idx_rpm) * 0.01;
    voltage2 = esc_data.ESC_TELEMETRY_1_TO_4_voltage_2(idx_rpm) * 0.01;
    current2 = esc_data.ESC_TELEMETRY_1_TO_4_current_2(idx_rpm) * 0.01;

    % psi_ref, alpha_ref and D_ref
    step_times = [72.5, 139, 159, 225];
    step_values = [0, pi/2, pi, 3*pi/2, 2*pi];
    
    % Psi
    psi_ref = zeros(size(time));
    
    for k = 1:length(step_times)
        psi_ref(time >= step_times(k)) = step_values(k+1);
    end
    
    % Alpha 
    alpha_min = -deg2rad(60);

    alpha_ref = zeros(size(time)); % Initialize
    
    % 1. Ramp down (0 -> -60°)
    idx1 = time >= step_times(1) & time < step_times(2);
    alpha_ref(idx1) = linspace(0, alpha_min, sum(idx1));
    
    % 2. Hold at -60°
    idx2 = time >= step_times(2) & time < step_times(3);
    alpha_ref(idx2) = alpha_min;
    
    % 3. Ramp up (-60° -> 0)
    idx3 = time >= step_times(3) & time < step_times(4);
    alpha_ref(idx3) = linspace(alpha_min, 0, sum(idx3));
    
    % 4. Hold at 0 after last step
    idx4 = time >= step_times(4);
    alpha_ref(idx4) = 0;

    % Depth
    D_min = 15;      % Max depth
    D_mid = 8;       % Mid depth
    step_times_D = [36, 74, 141.5, 161, 231.2, 251];
    
    
    D_ref = zeros(size(time));
    
    % 1. Ramp down: 0 -> 15 m
    idx1 = time >= step_times_D(1) & time < step_times_D(2);
    D_ref(idx1) = linspace(0, D_min, sum(idx1));
    
    % 2. Hold at 15 m
    idx2 = time >= step_times_D(2) & time < step_times_D(3);
    D_ref(idx2) = D_min;
    
    % 3. Ramp up: 15 -> 8 m
    idx3 = time >= step_times_D(3) & time < step_times_D(4);
    D_ref(idx3) = linspace(D_min, D_mid, sum(idx3));
    
    % 4. Hold at 8 m
    idx4 = time >= step_times_D(4) & time < step_times_D(5);
    D_ref(idx4) = D_mid;
    
    % 5. Ramp up: 8 -> 0 m
    idx5 = time >= step_times_D(5) & time < step_times_D(6);
    D_ref(idx5) = linspace(D_mid, 0, sum(idx5));
    
    % % Ensure D_ref holds at zero after the last ramp
    idx6 = time > step_times_D(6);
    D_ref(idx6) = 0;


    % Assign all variables to base workspace
    assignin('base', 'belt_1_speed', belt_1_speed);
    assignin('base', 'belt_2_speed', belt_2_speed);
    assignin('base', 'belt_1_rpm', belt_1_rpm);
    assignin('base', 'belt_2_rpm', belt_2_rpm);

    assignin('base', 'mag1_x', mag1_x);
    assignin('base', 'mag1_y', mag1_y);
    assignin('base', 'mag1_z', mag1_z);
    assignin('base', 'acc_x', acc_x);
    assignin('base', 'acc_y', acc_y);
    assignin('base', 'acc_z', acc_z);
    assignin('base', 'mag2_x', mag2_x);
    assignin('base', 'mag2_y', mag2_y);
    assignin('base', 'mag2_z', mag2_z);

    assignin('base', 'depth_meas', depth_meas);

    assignin('base', 'belt_1_dist', belt_1_dist);
    assignin('base', 'belt_2_dist', belt_2_dist);
    assignin('base', 'belt_1_speed_wheel', belt_1_speed_wheel);
    assignin('base', 'belt_2_speed_wheel', belt_2_speed_wheel);

    assignin('base', 'voltage1', voltage1);
    assignin('base', 'current1', current1);
    assignin('base', 'voltage2', voltage2);
    assignin('base', 'current2', current2);

    assignin('base', 'time_rpm', time_rpm);
    assignin('base', 'time_imu', time_imu);
    assignin('base', 'time_mag2', time_mag2);
    assignin('base', 'time_pos', time_pos);
    assignin('base', 'time_wheel', time_wheel);
    assignin('base', 'time_esc', time_esc);
    assignin('base', 'time', time);
    assignin('base', 'psi_ref', psi_ref);
    assignin('base', 'alpha_ref', alpha_ref);
    assignin('base', 'D_ref', D_ref);

    disp(['Loaded data for TEST' num2str(test_num) ' | Indices: ' num2str(idx_window_start) ' to ' num2str(idx_window_end)]);
end
