function X_pid = Xpid_error_init(reset_pid, OUT_FREQ, CUT_OFF_FREQ_POS)
    
    %% Input Arguments
    % reset_c : This is used to reset the errors of the PID controller.
    % OUT_FREQ : The frequency at which the low pass filter should run.
    % CUT_OFF_FREQ_POS: Cutt off frequency for low pass filter.
    %% Ouput Argument
    % X_pid:  A struct containing the errors for the PID controller.
    
    
    %% Declare the variable as persistent/static
    persistent x_curr_error
    persistent x_cumm_error
    persistent vx_error
    persistent ax_error
    persistent x_prev_error
    
    % struct containing the information and paramets for the low pass filter.
    lpf_data = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, 0.0);
    
    if reset_pid == 1
        x_curr_error = 0.0;
        x_cumm_error = 0.0;
        vx_error = 0.0;
        ax_error = 0.0;
        x_prev_error = 0.0;
    end
    
    X_pid.x_curr_error = x_curr_error;
    X_pid.x_cumm_error = x_cumm_error;
    X_pid.vx_error = vx_error;
    X_pid.ax_error = ax_error;
    X_pid.x_prev_error = x_prev_error;
    X_pid.lpf_data = lpf_data;
end

