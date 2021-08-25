function Y_pid = Ypid_error_init(reset_pid)
    %% Input Arguments
    % reset_pid : This is used to reset the errors of the PID controller.
    
    %% Ouput Argument
    % Y_pid:  A struct containing the errors for the PID controller.
    
    
    %% Declare the variable as persistent/static
    persistent y_curr_error
    persistent y_cumm_error
    persistent vy_error
    
    if reset_pid == 1
        y_curr_error = 0.0;
        y_cumm_error = 0.0;
        vy_error = 0.0;
    end
    
    Y_pid.y_curr_error = y_curr_error;
    Y_pid.y_cumm_error = y_cumm_error;
    Y_pid.vy_error = vy_error;
end

