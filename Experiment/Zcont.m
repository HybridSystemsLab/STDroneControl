function [T, output, Z_pid_err] = Zcont(Z_pid, z_d, z, v_z, v_z_d, dt)
    
    %% Input Arguments
    % Z_pid : A struct containing the errors of the Z position PID controller.
    % z_d: desired z position
    % z: current z position
    % v_z: current velcoity in z
    % v_z_d: desired velocity in z
    % dt: sampling time
    
    %% Ouput Argument
    % T:  Thrust that is needed to be commanded to the quadrotor/drone.
    % output: output of the Z-pos PID controller before clipping to MAX ACCELERATION.
    % Z_pid_err : A struct containing the updated errors of the Z position PID controller.
    
    
    output = 0.0;
    MAX_OUT = 10; % Acceleration limitation
    MAX_T = 255; % Needed for conversion to 0 - 255
    
    %% Gains
     %% fast gains
%     K_p_z = 7.0;
%     K_i_z = 7.5;
%     K_d_z = 1.25;

%     K_p_z = 0.1*7.0;
%     K_i_z = 0.1*7.5;
%     K_d_z = 0.1*1.25;
%     
    %% slow gains
%     K_p_z = 7.5; %0.6*7.0;
%     K_i_z = 7.5; %0.4*7.5
%     K_d_z = 1.25 ; % 1.25;
    
    %% faster gains
%     K_p_z = 1.5*7.5; %0.6*7.0;
%     K_i_z = 2*7.5; %0.4*7.5
%     K_d_z = 1.5*1.25 ; % 1.25
    K_p_z = 1*7.5; %0.6*7.0;
    K_i_z = 2*7.5; %0.4*7.5
    K_d_z = 1*1.25 ; % 1.25
    
    
    Z_pid_err.z_curr_error = z_d - z;
    output = output + K_p_z*Z_pid_err.z_curr_error; % 10.0
    
    Z_pid_err.z_cumm_error = Z_pid.z_cumm_error + Z_pid_err.z_curr_error*dt;
    output = output + K_i_z * Z_pid_err.z_cumm_error;
    
    
    Z_pid_err.vz_error = v_z_d - v_z;
    output = output + K_d_z * Z_pid_err.vz_error;
    
    % Need to convert to 0-255
    output_n = min(max(0, output), MAX_OUT);
    output_n = output_n*(MAX_T/MAX_OUT);
    %convert the value to 0 - 255
    T = uint8(output_n);
end