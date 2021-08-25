function [x_acc_d, output_x, X_pid_err, y_acc_d, output_y, Y_pid_err] = position_controller(X_pid, x_d, x, v_x_d, Y_pid, y_d, y, v_y_d, dt)
    %% Input Arguments
    % X_pid : A struct containing the errors of the X position PID controller.
    % x_d: desired x position
    % x: current x position
    % v_x: current velcoity in x
    % v_x_d: desired velocity in x
    % Y_pid : A struct containing the errors of the Y position PID controller.
    % y_d: desired y position
    % y: current y position
    % v_y: current velcoity in y
    % v_y_d: desired velocity in y
    % dt: sampling time
    
    %% Ouput Argument
    % x_acc_d:  Desired accelration in x.
    % output_x: output of the X-pos PID controller before clipping to MAX ACCELERATION.
    % X_pid_err : A struct containing the updated errors of the X position PID controller.
    % y_acc_d:  Desired accelration in y.
    % output_y: output of the Y-pos PID controller before clipping to MAX ACCELERATION.
    % Y_pid_err : A struct containing the updated errors of the Y position PID controller.
    
    output1 = 0.0;
    output_x = 0.0;
    MAX_ACC = 2.0;
    
%% ============================X-POS CONTROLLER================================
    
    %% -------------------- FIRST PID BLOCK ------------------------------
    % Gains
    K_p_x1 = 4;
    K_i_x1 = 0.0;
    K_d_x1 = 0.0;
    
    X_pid_err.x_curr_error = x_d - x;
    output1 = output1 + K_p_x1*X_pid_err.x_curr_error; 
    
    %% -------------------- SECOND PID BLOCK ------------------------------
    % Gains
    K_p_x2 = 0.4;
    K_i_x2 = 0.4;
    K_d_x2 = 0.6;
    
    X_pid_err.vx_curr_error = v_x_d - output1;
    output_x = output_x + K_p_x2*X_pid_err.vx_curr_error; 
    
    X_pid_err.deriv = (X_pid_err.vx_curr_error - X_pid.vx_prev)/dt;
    [deriv_f, X_pid_err.lpf_data] = lpf_2(X_pid.lpf_data, X_pid_err.deriv);
    X_pid_err.deriv = deriv_f;
    output_x = output_x + K_d_x2 * X_pid_err.deriv;
    
    X_pid_err.vx_cumm_error = X_pid.vx_cumm_error + X_pid_err.vx_curr_error*dt;
    X_pid_err.vx_cumm_error = max(min(X_pid_err.vx_cumm_error, 0.4), -0.4);
    output_x = output_x + K_i_x2 * X_pid_err.vx_cumm_error;
    
    X_pid_err.vx_prev = X_pid_err.vx_curr_error;
    
    % Need to convert to clip the accelearation
    output_x = min(max(-MAX_ACC, output_x), MAX_ACC);
    x_acc_d = output_x;
    
    
    
%% ============================Y-POS CONTROLLER================================
    output1 = 0.0;
    output_y = 0.0;
    
    
    %% -------------------- FIRST PID BLOCK ------------------------------
    % Gains
    K_p_y1 = K_p_x1;
    K_i_y1 = 0.0;
    K_d_y1 = 0.0;
    
    Y_pid_err.y_curr_error = y_d - y;
    output1 = output1 + K_p_y1*Y_pid_err.y_curr_error; 
    
    %% -------------------- SECOND PID BLOCK ------------------------------
    % Gains
    K_p_y2 = K_p_x2;
    K_i_y2 = K_i_x2;
    K_d_y2 = K_d_x2;
    
    Y_pid_err.vy_curr_error = v_y_d - output1;
    output_y = output_y + K_p_y2*Y_pid_err.vy_curr_error; 
    
    Y_pid_err.deriv = (Y_pid_err.vy_curr_error - Y_pid.vy_prev)/dt;
    [deriv_f, Y_pid_err.lpf_data] = lpf_2(Y_pid.lpf_data, Y_pid_err.deriv);
    Y_pid_err.deriv = deriv_f;
    output_y = output_y + K_d_y2 * Y_pid_err.deriv;
    
    Y_pid_err.vy_cumm_error = Y_pid.vy_cumm_error + Y_pid_err.vy_curr_error*dt;
    Y_pid_err.vy_cumm_error = max(min(Y_pid_err.vy_cumm_error, 0.4), -0.4);
    output_y = output_y + K_i_y2 * Y_pid_err.vy_cumm_error;
    
    Y_pid_err.vy_prev = Y_pid_err.vy_curr_error;
    
    % Need to convert to clip the accelearation
    output_y = min(max(-MAX_ACC, output_y), MAX_ACC);
    y_acc_d = output_y;
    
end