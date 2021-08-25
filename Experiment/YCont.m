function [y_acc_d, output, Y_pid_err] = YCont(Y_pid, y_d, y, v_y, v_y_d, dt)
    %% Input Arguments
    % Y_pid : A struct containing the errors of the Y position PID controller.
    % y_d: desired y position
    % y: current y position
    % v_y: current velcoity in y
    % v_y_d: desired velocity in y
    % dt: sampling time
    
    %% Ouput Argument
    % y_acc_d:  Desired accelration in y.
    % output: output of the Y-pos PID controller before clipping to MAX ACCELERATION.
    % Y_pid_err : A struct containing the updated errors of the Y position PID controller.
    
    output = 0.0;
    MAX_ACC = 2.0; % Limit the acceleration to 2 m/s/s
    
    
    %% Gains
    K_p_y = 0*0.15;
    K_i_y = 0*1.0;
    K_d_y = 0*2.0;
    
    
    Y_pid_err.y_curr_error = y_d - y;
    output = output + K_p_y*Y_pid_err.y_curr_error; % 10.0
    
    Y_pid_err.y_cumm_error = Y_pid.y_cumm_error + Y_pid_err.y_curr_error*dt;
    output = output + K_i_y * Y_pid_err.y_cumm_error;
    
    
    Y_pid_err.vy_error = v_y_d - v_y;
    output = output + K_d_y * Y_pid_err.vy_error;
    
    % Limit the acceleration
    y_acc_d = min(max(-MAX_ACC, output), MAX_ACC); 
end