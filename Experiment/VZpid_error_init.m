function Z_pid = VZpid_error_init(reset_pid, OUT_FREQ, CUT_OFF_FREQ_POS)
    %% Input Arguments
    % reset_pid : This is used to reset the errors of the PID controller.
    % OUT_FREQ : The frequency at which the low pass filter should run.
    % CUT_OFF_FREQ_POS: Cutt off frequency for low pass filter.
    %% Ouput Argument
    % Z_pid:  A struct containing the errors for the PID controller.
    
    
    %% Declare the variable as persistent/static
    persistent z_curr_error
    persistent z_cumm_error
    persistent vz_curr_error
    persistent vz_cumm_error
    persistent vz_prev
    persistent deriv
    
    % struct containing the information and paramets for the low pass filter.
    lpf_data = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, 0.0);
    
    if reset_pid == 1
        z_curr_error = 0.0;
        z_cumm_error = 0.0;
        vz_curr_error = 0.0;
        vz_cumm_error = 0.0;
        vz_prev = 0.0;
        deriv = 0.0;
    end
    
    Z_pid.z_curr_error = z_curr_error;
    Z_pid.z_cumm_error = z_cumm_error;
    Z_pid.vz_curr_error = vz_curr_error;
    Z_pid.vz_cumm_error = vz_cumm_error;
    Z_pid.vz_prev = vz_prev;
    Z_pid.deriv = deriv;
    Z_pid.lpf_data = lpf_data;
end

