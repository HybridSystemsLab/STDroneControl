function [M, att_errors] = attitudeController2(att_err, state, desiredAngles, dt)
    %% Input Arguments
    % att_err : A struct containing the errors of the attitude PID controller.
    % state: states of the quadrotor/drone
    % desiredAngles: struct containing the desired angles to track.
    % dt: sampling time
    
    %% Ouput Argument
    % M:  3x1 vector ([tau_p; tau_q; tau_r]) containing the inputs for the quadrotor/drone.
    % att_errors : A struct containing the updated errors of the attitude PID controller.

    att_errors = att_error_init(1);
    % CONSTANTS
    MAX_ANGULAR_RATE = 2; % 4 degrees/s
    %% ==== THETA ====

    %------- GAINZZZZ -------
    %K_p_x = 5;
    %Note:- Before changing the inertia to kg-m^2 the gains were (10, 1.05, 2.5)
    K_p_theta = 0.01; %0.0009;
    K_i_theta = 0.0;% 0.0;
    K_d_theta = 2.5e-3;%0.001;



    % ------ FIRST BLOCK ------
    error_theta = desiredAngles.theta_d - state.theta;
    error_wx = (desiredAngles.theta_d - att_err.prev_des_theta)/dt - state.w_x;
    max_theta_rate = max(min(error_wx, MAX_ANGULAR_RATE), -MAX_ANGULAR_RATE);
    att_errors.prev_des_theta = desiredAngles.theta_d;

    att_errors.wx_cumm_error = att_err.wx_cumm_error + error_theta*dt;
    t_p = (K_p_theta*error_theta) + (K_i_theta*att_errors.wx_cumm_error) +  (K_d_theta*max_theta_rate);
    %wx_prev_error = error_wx;

    %% ==== PHI ====
    %------- GAINZZZZ -------
    %Note:- Before changing the inertia to kg-m^2 the gains were (20, 0.25, 3.75)
    K_p_phi = K_p_theta; %5e-3; %0.005;
    K_i_phi = K_i_theta;
    K_d_phi = K_d_theta; %1e-3; %0.001;

    % Second NOTE:- using kp = 0.002 and kd = 0.001 gives no overshoot

    % ------ FIRST BLOCK ------
    error_phi = desiredAngles.phi_d - state.phi;
    error_wy = (desiredAngles.phi_d - att_err.prev_des_phi)/dt - state.w_y;
    max_phi_rate = max(min(error_wy, MAX_ANGULAR_RATE), -MAX_ANGULAR_RATE);
    att_errors.prev_des_phi = desiredAngles.phi_d;

    att_errors.wy_cumm_error = att_err.wy_cumm_error + error_phi*dt;
    t_q = (K_p_phi*error_phi) + (K_i_phi*att_errors.wy_cumm_error) +  (K_d_phi*max_phi_rate);

    %% ==== PSI ====
    %------- GAINZZZZ -------

    %Note:- Before changing the inertia to kg-m^2 the gains were (10, 0.0125, 3.75)
    K_p_psi = 0.003;%10;  %10
    K_i_psi = 0.0;%0.0125; %0.125
    K_d_psi = 0.00125;%3.75;  %3.0

    % ------ FIRST BLOCK ------
    error_psi = desiredAngles.psi_d - state.psi;
    error_wz = (desiredAngles.psi_d - att_err.prev_des_psi)/dt - state.w_z;
    max_psi_rate = max(min(error_wz, MAX_ANGULAR_RATE), -MAX_ANGULAR_RATE);
    att_errors.prev_des_psi = desiredAngles.psi_d;

    att_errors.wz_cumm_error = att_err.wz_cumm_error + error_psi*dt;
    t_r = (K_p_psi*error_psi) + (K_i_psi*att_errors.wz_cumm_error) +  (K_d_psi*max_psi_rate);



    %% COMBINE THE INPUTS INTO M
    M = [t_p; t_q; t_r];
end

