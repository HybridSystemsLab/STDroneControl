function Y_pid = VYpid_error_init(reset_pid, OUT_FREQ, CUT_OFF_FREQ_POS)
    %% Input Arguments
    % reset_pid : This is used to reset the errors of the PID controller.
    % OUT_FREQ : The frequency at which the low pass filter should run.
    % CUT_OFF_FREQ_POS: Cutt off frequency for low pass filter.
    %% Ouput Argument
    % Y_pid:  A struct containing the errors for the PID controller.
    
    
    %% Declare the variable as persistent/static
    persistent y_curr_error
    persistent vy_curr_error
    persistent vy_cumm_error
    persistent vy_prev
    persistent deriv
    
    % struct containing the information and paramets for the low pass filter.
    lpf_data = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, 0.0);
    
    if reset_pid == 1
        y_curr_error = 0.0;
        vy_curr_error = 0.0;
        vy_cumm_error = 0.0;
        vy_prev = 0.0;
        deriv = 0.0;
    end
    
    Y_pid.y_curr_error = y_curr_error;
    Y_pid.vy_curr_error = vy_curr_error;
    Y_pid.vy_cumm_error = vy_cumm_error;
    Y_pid.vy_prev = vy_prev;
    Y_pid.deriv = deriv;
    Y_pid.lpf_data = lpf_data;
end

