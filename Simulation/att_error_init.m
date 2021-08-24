function att_err = att_error_init(reset_c)
    %% Input Arguments
    % reset_c : This is used to reset the errors of the PID controller.
    
    %% Ouput Argument
    % att_err:  A struct containing the errors for the PID controller.
    
    
    %% Declare the variable as persistent/static
    persistent wx_cumm_error
    persistent wx_prev_error
    persistent x_prev_deriv
    persistent prev_des_theta

    persistent wy_cumm_error
    persistent wy_prev_error
    persistent y_prev_deriv
    persistent prev_des_phi

    persistent wz_cumm_error
    persistent wz_prev_error
    persistent z_prev_deriv
    persistent prev_des_psi

    %% Reset the errors if needed
    if reset_c == 1
        z_prev_deriv = 0.0;
        wz_prev_error = 0.0;
        wz_cumm_error = 0.0;
        prev_des_theta = 0.0;

        y_prev_deriv = 0.0;
        wy_prev_error = 0.0;
        wy_cumm_error = 0.0;
        prev_des_phi = 0.0;

        x_prev_deriv = 0.0;
        wx_prev_error = 0.0;
        wx_cumm_error = 0.0;
        prev_des_psi = 0.0;

    end



    %% Set the att_err struct
    att_err.wx_cumm_error = wx_cumm_error;
    att_err.wx_prev_error = wx_prev_error;
    att_err.x_prev_deriv = x_prev_deriv;

    att_err.wy_cumm_error = wy_cumm_error;
    att_err.wy_prev_error = wy_prev_error;
    att_err.y_prev_deriv = y_prev_deriv;

    att_err.wz_cumm_error = wz_cumm_error;
    att_err.wz_prev_error = wz_prev_error;
    att_err.z_prev_deriv = z_prev_deriv;


    att_err.prev_des_theta = prev_des_theta;
    att_err.prev_des_phi = prev_des_phi;
    att_err.prev_des_psi = prev_des_psi;
end