function Z_pid = Zpid_error_init(reset_pid)
    %% Input Arguments
    % reset_pid : This is used to reset the errors of the PID controller.
    
    %% Ouput Argument
    % Z_pid:  A struct containing the errors for the PID controller.
    
    
    %% Declare the variable as persistent/static
    persistent z_curr_error
    persistent z_cumm_error
    persistent vz_error
    
    if reset_pid == 1
        z_curr_error = 0.0;
        z_cumm_error = 0.0;
        vz_error = 0.0;
    end
    
    Z_pid.z_curr_error = z_curr_error;
    Z_pid.z_cumm_error = z_cumm_error;
    Z_pid.vz_error = vz_error;
end

