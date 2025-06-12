%% Load Test Rig Data
% Loads all necessary data from the test rig experiments
% for running the EKF navigation algorithm.


%% ===================== Test definition =====================
clear;

test_num = 3;

% TEST 2
% idx_start = 2; % Set to 1 for MATLAB indexing
% idx_end = 2350;

% TEST 3
idx_start = 1000; % Set to 1 for MATLAB indexing
idx_end = 2500;

load_test_data_RIG(test_num, idx_start, idx_end);


%% ===================== Plotting params =====================
label_size = 20;
axis_size = 15;
title_size = 23;
legend_size = 15;
legend_size_F3_and_4 = 12;
grayColor = [0.25 0.25 0.25];


%% ===================== Load Data Function =====================
function load_test_data_RIG(test_num, idx_window_start, idx_window_end)
    % Constants
    rot_ang      = deg2rad(90);
    Rot_x_neg_90 = [1, 0,             0;
                    0, cos(rot_ang), -sin(rot_ang);
                    0, sin(rot_ang),  cos(rot_ang)];

    % Create folder name string based on test number
    folder_name = sprintf('TEST%d', test_num);

    % File list
    imu_file        = fullfile(folder_name, sprintf('TEST%d_IMU.csv', test_num));
    mag2_file       = fullfile(folder_name, sprintf('TEST%d_MAG2.csv', test_num));
    speeds_file     = fullfile(folder_name, sprintf('TEST%d_SPEEDS.csv', test_num));
    alpha_ref_file  = fullfile(folder_name, 'TEST3_ALPHA_AND_PSI_REF.csv');
    pos_file      = fullfile(folder_name, sprintf('TEST%d_DEPTH.csv', test_num));

    % Load files
    imu_data      = readtable(imu_file);
    mag2_data     = readtable(mag2_file);
    speeds_data   = readtable(speeds_file);
    alpha_psi_ref_data = readtable(alpha_ref_file);
    pos_data      = readtable(pos_file);

    % Lengths for windowing
    n_imu    = height(imu_data);
    n_mag2   = height(mag2_data);
    n_speeds = height(speeds_data);
    n_alpha  = height(alpha_psi_ref_data);
    n_pos    = height(pos_data);

    % Default: full window for each signal
    if nargin < 2, idx_window_start = 1; end
    if nargin < 3
        idx_window_end = min([n_imu, n_mag2, n_speeds, n_alpha, n_pos]);
    end

    % Clamp end index for each signal
    idx_imu     = idx_window_start : min(idx_window_end, n_imu);
    idx_mag2    = idx_window_start : min(idx_window_end, n_mag2);
    idx_speeds  = idx_window_start : min(idx_window_end, n_speeds);
    idx_alpha_psi   = idx_window_start : min(idx_window_end, n_alpha);
    idx_pos   = idx_window_start : min(idx_window_end, n_pos);

    % Time variables
    ts_imu     = imu_data.timestamp(idx_imu);
    time_imu   = ts_imu - ts_imu(1);

    ts_mag2    = mag2_data.timestamp(idx_mag2);
    time_mag2  = ts_mag2 - ts_mag2(1);

    ts_speeds  = speeds_data.timestamp(idx_speeds);
    time_speeds = ts_speeds - ts_speeds(1);

    ts_alpha   = alpha_psi_ref_data.timestamp(idx_alpha_psi);
    time_alpha = ts_alpha - ts_alpha(1);
    
    ts_pos   = pos_data.timestamp(idx_pos);
    time_pos = ts_pos - ts_pos(1);

    % Synchronize time
    N = min([length(time_imu), length(time_mag2), length(time_speeds), length(time_alpha)]);
    time = time_imu(1:N);

    % Mag1 from IMU (normalized, rotated)
    mag1_raw = [imu_data.RAW_IMU_xmag(idx_imu), imu_data.RAW_IMU_ymag(idx_imu), imu_data.RAW_IMU_zmag(idx_imu)]';
    mag1_norm = sqrt(sum(mag1_raw.^2, 1));
    mag1_unit = mag1_raw ./ mag1_norm;
    mag1_rot = Rot_x_neg_90 * mag1_unit;
    mag1_x = mag1_rot(1, :)';
    mag1_y = mag1_rot(2, :)';
    mag1_z = mag1_rot(3, :)';

    % Acc from IMU (rotated)
    acc_raw = [imu_data.RAW_IMU_xacc(idx_imu)'; imu_data.RAW_IMU_yacc(idx_imu)'; imu_data.RAW_IMU_zacc(idx_imu)'] * (-9.81/1000);
    acc_rot = Rot_x_neg_90 * acc_raw;
    acc_x = acc_rot(1, :)';
    acc_y = acc_rot(2, :)';
    acc_z = acc_rot(3, :)';

    % Mag2 from MAG2 file (normalized, rotated)
    mag2_raw = [mag2_data.SCALED_IMU2_xmag(idx_mag2), mag2_data.SCALED_IMU2_ymag(idx_mag2), mag2_data.SCALED_IMU2_zmag(idx_mag2)]';
    mag2_norm = sqrt(sum(mag2_raw.^2, 1));
    mag2_unit = mag2_raw ./ mag2_norm;
    mag2_rot = Rot_x_neg_90 * mag2_unit;
    mag2_x = mag2_rot(1, :)';
    mag2_y = mag2_rot(2, :)';
    mag2_z = mag2_rot(3, :)';

    % Collect u, r
    u = speeds_data.u(idx_speeds);
    r = speeds_data.r(idx_speeds);

    % Collect Depth measurement
    depth_meas = pos_data.GLOBAL_POSITION_INT_relative_alt(idx_pos) * (-1 / 1000);

    % Alpha ref from CSV
    alpha_ref = alpha_psi_ref_data.alpha_ref(idx_alpha_psi);
    psi_ref = alpha_psi_ref_data.psi_ref(idx_alpha_psi);
    D_ref = alpha_psi_ref_data.D_ref(idx_alpha_psi);

    % Assign all variables to base workspace
    assignin('base', 'mag1_x', mag1_x(1:N));
    assignin('base', 'mag1_y', mag1_y(1:N));
    assignin('base', 'mag1_z', mag1_z(1:N));
    assignin('base', 'acc_x', acc_x(1:N));
    assignin('base', 'acc_y', acc_y(1:N));
    assignin('base', 'acc_z', acc_z(1:N));
    assignin('base', 'mag2_x', mag2_x(1:N));
    assignin('base', 'mag2_y', mag2_y(1:N));
    assignin('base', 'mag2_z', mag2_z(1:N));
    assignin('base', 'time_imu', time_imu(1:N));
    assignin('base', 'time_mag2', time_mag2(1:N));
    assignin('base', 'time_speeds', time_speeds(1:N));
    assignin('base', 'time_alpha', time_alpha(1:N));
    assignin('base', 'time', time);
    assignin('base', 'u', u(1:N));
    assignin('base', 'r', r(1:N));
    assignin('base', 'alpha_ref', alpha_ref(1:N));
    assignin('base', 'psi_ref', psi_ref(1:N));
    assignin('base', 'D_ref', D_ref(1:N));
    assignin('base', 'depth_meas', depth_meas);

    disp(['Loaded IMU, MAG2, SPEEDS, and ALPHA_REF data for TEST' num2str(test_num) ...
         ' | Indices: ' num2str(idx_window_start) ' to ' num2str(idx_window_end)]);
end
