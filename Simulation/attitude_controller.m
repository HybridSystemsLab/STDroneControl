function [M, att_errors] = attitude_controller(att_err, state, desiredAngles, dt)
    %% Input Arguments
    % att_err : A struct containing the errors of the attitude PID controller.
    % state: states of the quadrotor/drone
    % desiredAngles: struct containing the desired angles to track.
    % dt: sampling time
    
    %% Ouput Argument
    % M:  3x1 vector ([tau_p; tau_q; tau_r]) containing the inputs for the quadrotor/drone.
    % att_errors : A struct containing the updated errors of the attitude PID controller.
    

    %% ==== THETA ====
    att_errors = att_error_init(1);
    %------- GAINZZZZ -------
    K_p_x = 8;%5;
    K_p_theta = 0.08;%15;
    K_i_theta = 0;%0;
    K_d_theta = 0.0;%0.01;

    % ------ FIRST BLOCK ------
    error_theta = desiredAngles.theta_d - state.theta;
    x_s1 = K_p_x * error_theta; 

    % ------ SECOND BLOCK ------
    error_wx = x_s1 - state.w_x;
    att_errors.wx_cumm_error = att_err.wx_cumm_error + error_wx*dt;
    deriv_x = (error_wx-att_err.wx_prev_error)/dt;
    deriv_x = att_err.x_prev_deriv + (deriv_x - att_err.x_prev_deriv)*0.025;
    t_p = (K_p_theta*error_wx) + (K_i_theta*att_errors.wx_cumm_error) +  (K_d_theta*deriv_x);
    att_errors.wx_prev_error = error_wx;
    att_errors.x_prev_deriv = deriv_x;


    %% ==== PHI ====
    %------- GAINZZZZ -------
    K_p_y = K_p_x; %5;
    K_p_phi = K_p_theta;%10;
    K_i_phi = 0;%0;
    K_d_phi = 0;%0.1;

    % ------ FIRST BLOCK ------
    error_phi = desiredAngles.phi_d - state.phi;
    x_s2 = K_p_y * error_phi; 

    % ------ SECOND BLOCK ------
    error_wy = x_s2 - state.w_y;
    att_errors.wy_cumm_error = att_err.wy_cumm_error + error_wy*dt;
    deriv_y = (error_wy - att_err.wy_prev_error)/dt;
    deriv_y = att_err.y_prev_deriv + (deriv_y - att_err.y_prev_deriv)*0.025; % 0.025 is a filter coefficient
    t_q = (K_p_phi*error_wy) + (K_i_phi*att_errors.wy_cumm_error) +  (K_d_phi*deriv_y);
    att_errors.wy_prev_error = error_wy;
    att_errors.y_prev_deriv = deriv_y;

    %% ==== PSI ====
    %------- GAINZZZZ -------
    K_p_z = 5;%8;
    K_p_psi = 0.09;%10;
    K_i_psi = 0;%0;
    K_d_psi = 0;%0.1;

    % ------ FIRST BLOCK ------
    error_psi = desiredAngles.psi_d - state.psi;
    x_s3 = K_p_z * error_psi; 

    % ------ SECOND BLOCK ------
    error_wz = x_s3 - state.w_z;
    att_errors.wz_cumm_error = att_err.wz_cumm_error + error_wz*dt;
    deriv_z = (error_wz - att_err.wz_prev_error)/dt;
    t_r = (K_p_psi*error_wz) + (K_i_psi*att_errors.wz_cumm_error) +  (K_d_psi*deriv_z);
    att_errors.wz_prev_error = error_wz;


    %% COMBINE THE INPUTS INTO M
    M = [t_p; t_q; t_r];
end

