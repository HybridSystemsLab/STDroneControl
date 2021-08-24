function pos_err = pos_error_init(reset_c)
    
    %% Input Arguments
    % reset_c : This is used to reset the errors of the PID controller.
    
    %% Ouput Argument
    % pos_err:  A struct containing the errors for the PID controller.
    
    
    %% Declare the variable as persistent/static
    persistent x_curr_error
    persistent x_cumm_error
    persistent x_prev_des

    persistent y_curr_error
    persistent y_cumm_error
    persistent y_prev_des

    persistent z_curr_error
    persistent z_cumm_error
    persistent z_prev_des

    %% Reset if needed
    if reset_c == 1
       x_curr_error = 0.0;
       x_cumm_error = 0.0;
       x_prev_des = 0.0;

       y_curr_error = 0.0;
       y_cumm_error = 0.0;
       y_prev_des = 0.0;

       z_curr_error = 0.0;
       z_cumm_error = 0.0;
       z_prev_des = 0.0;
    end

%% Set the pos_err struct
    pos_err.x_curr_error = x_curr_error;
    pos_err.x_cumm_error = x_cumm_error;
    pos_err.x_prev_des = x_prev_des;
    
    pos_err.y_curr_error = y_curr_error;
    pos_err.y_cumm_error = y_cumm_error;
    pos_err.y_prev_des = y_prev_des;
    
    pos_err.z_curr_error = z_curr_error;
    pos_err.z_cumm_error = z_cumm_error;
    pos_err.z_prev_des = z_prev_des;
end