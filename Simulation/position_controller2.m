function [T, theta_d, phi_d, p_errors] = position_controller2(pos_err, state, des_pos, dt, params)
    %% Input Arguments
    % pos_err : A struct containing the errors of the attitude PID controller.
    % state: states of the quadrotor/drone
    % des_pos: struct containing the desired position to track.
    % dt: sampling time
    % params: A struct containing the parameters of the quadrotor/drone
    
    %% Ouput Argument
    % T:  Input thrust(t) for the quadrotor/drone.
    % theta_d: desired theta angle for the quadrotor to track.
    % phi_d: desired phi angle for the quadrotor to track.
    % p_errors : A struct containing the updated errors of the position PID controller.

    p_errors = pos_error_init(1);
    %% CONSTATNS
    MAX_ANGLE = pi/6;

    %% Vertical Controller
        K_p = 12.0;   %10.0;  %4.0
        K_i = 1.25;   %10.0;  %5.0
        K_d = 4.0;   %2.5;  %1.0

        p_errors.z_curr_error = (des_pos(3) - state.z); %conver to cm error *100 missing
        vz_error = (des_pos(3) - pos_err.z_prev_des)/dt - state.vz;
        p_errors.z_cumm_error = pos_err.z_cumm_error + p_errors.z_curr_error*dt;
        T = (K_p*p_errors.z_curr_error) + (K_i*p_errors.z_cumm_error) + (K_d*vz_error);
        p_errors.z_prev_des = des_pos(3);
        T = max(0.001, min(T, 1.5));


    %% X-pos Controller
        % -------- Gainzzz --------
        K_p_x = 12.0;  %0.1 OC       %0.05
        K_i_x = 0.001;  %0.1 OC       %0.5
        K_d_x = 9.0;  %7 0.15 OC      %40.0

        p_errors.x_curr_error = (des_pos(1) - state.x);% * ROLL_OFF;
        vx_curr_error = (des_pos(1) - pos_err.x_prev_des)/dt - state.vx;
        p_errors.x_cumm_error = pos_err.x_cumm_error + p_errors.x_curr_error*dt;
        roll_ref = (K_p_x*p_errors.x_curr_error) + (K_i_x*p_errors.x_cumm_error) + (K_d_x*vx_curr_error);
        p_errors.x_prev_des = des_pos(1);
        

    %% Y-pos Controller
        % -------- Gainzzz --------
        K_p_y = 12.0; %15.0
        K_i_y = 0.001;
        K_d_y = 9.0;

        p_errors.y_curr_error = (des_pos(2) - state.y);
        vy_curr_error =  (des_pos(2) - pos_err.y_prev_des)/dt - state.vy;
        p_errors.y_cumm_error = pos_err.y_cumm_error + p_errors.y_curr_error*dt;
        pitch_ref = (K_p_y * p_errors.y_curr_error) + (K_i_y * p_errors.y_cumm_error) + (K_d_y * vy_curr_error);
        p_errors.y_prev_des = des_pos(2);

     %% Convert from desired accelration to desired angles
        R_a_angle = [sin(des_pos(4)) cos(des_pos(4)); -cos(des_pos(4)) sin(des_pos(4))];
        ref = R_a_angle\((params.m/T) * [roll_ref; pitch_ref]);

        theta_d = min(max(ref(1), -MAX_ANGLE), MAX_ANGLE);
        phi_d = min(max(ref(2), -MAX_ANGLE), MAX_ANGLE);


end

