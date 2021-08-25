function [x_acc_d, output, X_pid_err] = XCont(X_pid, x_d, x, v_x, v_x_d, dt)
    %% Input Arguments
    % X_pid : A struct containing the errors of the X position PID controller.
    % x_d: desired x position
    % x: current x position
    % v_x: current velcoity in x
    % v_x_d: desired velocity in x
    % dt: sampling time
    
    %% Ouput Argument
    % x_acc_d:  Desired accelration in x.
    % output: output of the X-pos PID controller before clipping to MAX ACCELERATION.
    % X_pid_err : A struct containing the updated errors of the X position PID controller.

    output = 0.0;
    MAX_ACC = 2.0; % Limit the acceleration to 2 m/s/s
    
    %% Gains
    K_p_x = 3;%0.3*10.0; %0*0.15;
    K_i_x = 0.4;%1.2*10/1.5; %0*1.0;
    K_d_x = 2*1.125;%0.075*10*1.5; %0*2.0;
    K_dd_x = 0.0;
    
    X_pid_err.x_curr_error = x_d - x;
    output = output + K_p_x*X_pid_err.x_curr_error; % 10.0
    
%     X_pid_err.vx_error = (X_pid_err.x_curr_error - X_pid.x_prev_error)/dt;
%     [deriv_f, X_pid_err.lpf_data] = lpf_2(X_pid.lpf_data, X_pid_err.vx_error);
%     X_pid_err.vx_error = deriv_f;
%     output = output + K_d_x * X_pid_err.vx_error;
%     X_pid_err.x_prev_error = X_pid_err.x_curr_error;
    X_pid_err.vx_error = v_x_d - v_x;
    output = output + K_d_x * X_pid_err.vx_error;
    
    X_pid_err.x_cumm_error = X_pid.x_cumm_error + X_pid_err.x_curr_error*dt;
    output = output + K_i_x * X_pid_err.x_cumm_error;
    
    X_pid_err.ax_error = (X_pid_err.vx_error - X_pid.x_prev_error)/dt;
    [acc_f, X_pid_err.lpf_data] = lpf_2(X_pid.lpf_data, X_pid_err.ax_error);
    X_pid_err.ax_error = acc_f;
    output = output + K_dd_x * X_pid_err.ax_error;
    X_pid_err.x_prev_error = X_pid_err.vx_error;
    
    % Clip the acceleration
    x_acc_d = min(max(-MAX_ACC, output), MAX_ACC);
end