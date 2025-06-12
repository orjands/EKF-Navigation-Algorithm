%% Magnetometer Observation Model Calculations
% This script computes the expected magnetometer measurements based on the 
% current estimated state of the robot, including azimuth (α), heading (ψ), 
% and magnetic dip angle (θ).


%% ===================== Parameters =====================
clear;

syms r_c alpha psi
syms D u v


%% ===================== Rotation Matrix from BODY to NED =====================
% disp('----- NED frame unit vector presented in NED -----')
x_n = [1; 0; 0];
y_n = [0; 1; 0];
z_n = [0; 0; 1];

% disp('----- BODY frame unit vector when \alpha and \psi = 0 presented in NED -----')
x_0 = [1;  0;  0];
y_0 = [0; -1;  0];
z_0 = [0;  0; -1];

% First, we want to compute the rotation matrix from the robot_0 reference position, 
% which corresponds to the configuration where both α (alpha) and ψ (psi) are equal to zero.
disp('----- Rotation matrix from BODY_0_ref to NED -----')
c_pi = cos(pi);
s_pi = sin(pi);
s_pi = 0;

R_b0_2_ned = [c_pi, -s_pi, 0;
              s_pi,  c_pi, 0;
                 0,     0, 1]

% Then, determine the rotation from the BODY frame to the robot_0 reference frame.
disp('----- Rotation matrix from BODY to BODY_0_ref -----')

% First we rotate the robot back to its 0 psi ref!
% This is done by rotating the body -psi about the x axis of the robot!
R_neg_psi_about_x = [1,         0,          0;
                  0, cos(-psi), -sin(-psi);
                  0, sin(-psi),  cos(-psi)]

% Then we rotate the robot back to its 0 alpha ref!
% This is done by rotating the body alpha about the z axis of the robot!
R_alpha_about_z = [cos(alpha), -sin(alpha), 0;
                sin(alpha),  cos(alpha), 0;
                         0,           0, 1]

R_B_2_B_0 = R_alpha_about_z * R_neg_psi_about_x

% Then multiplies this to find the rotation matrix from BODY to NED
disp('----- Rotasjonsmatrise fra BODY til NED -----')
R_B2N = R_b0_2_ned * R_B_2_B_0

%% ===================== Observation Model Equation for EKF =====================
syms theta
n_ned = [cos(theta); 0; sin(theta)];
n_body = R_B2N.' * n_ned; % = R_sensor2body (=1) * R_mag2_to_sensor(=1) * (m / |m|)

mag_body = simplify((R_B2N.') * n_ned)

disp('----- h function and H matrix jacobian -----')
h1 = mag_body(1)
h2 = mag_body(2)
h3 = mag_body(3)

Jacobian_h1 = jacobian(h1, [alpha, D, psi, u, v])
Jacobian_h2 = jacobian(h2, [alpha, D, psi, u, v])
Jacobian_h3 = jacobian(h3, [alpha, D, psi, u, v])