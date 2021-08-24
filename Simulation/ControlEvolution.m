function [roll_plot, pitch_plot, yaw_plot, des_roll_plot, des_pitch_plot, des_yaw_plot, z_plot, x_plot, y_plot, x_error_plot, y_error_plot, z_error_plot, state_v, u_tt, error_p, error_a] = ControlEvolution(time_t, params, des, state, pos_err, att_err, logic_q, start_t)
    %% Input Arguments
    % time_t: Look ahead time
    % params: struct containing the quadrotor/drone parameters
    % des: struct containing the desired trajectory
    % state: struct containing the state of the quadrotor/drone.
    % pos_err: A struct containing the errors of the position PID controller.
    % att_err: A struct containing the errors of the attitude PID controller.
    % logic_q: Decision variable to choose between two controllers. 
    % start_t: Start time or the time at which we need to start tracking the desired trajectory

    %% Ouput Argument
    % roll_plot: Array containing the data of the roll state
    % pitch_plot: Array containing the data of the pitch state
    % yaw_plot: Array containing the data of the yaw state
    % des_roll_plot: Array containing the data of the desired roll
    % des_pitch_plot: Array containing the data of the desired pitch
    % des_yaw_plot: Array containing the data of the desired yaw
    % z_plot: Array containing the data of the x positon state
    % x_plot: Array containing the data of the y positon state
    % y_plot: Array containing the data of the z positon state
    % x_error_plot: Array containing the error along x state
    % y_error_plot: Array containing the error along y state
    % z_error_plot: Array containing the error along z state
    % state_v: An array of struct containing the updates states of the quadrotor/drone.
    % u_tt: An array of 4x1 input vector - [T, M] 
    % error_p: An array of struct containing the updated errors of the position PID controller.
    % error_a: An array of struct containing the updated errors of the attitude PID controller.

    
    
    ATTITUTDE_LOOP_RATE = 500; %Hz, Attitute Controller rate
    POSITION_LOOP_RATE = 50; %Hz , Position Controller rate

    dt = 1/ATTITUTDE_LOOP_RATE;
    dt_pos = 1/POSITION_LOOP_RATE;

    TT = time_t; % total simulation time

    time = 0:dt:TT; % time vector with step size dt to T
    sim_length = length(time);

    % Data for plotting purposes
    roll_plot = zeros(1,sim_length-2);
    pitch_plot = zeros(1,sim_length-2);
    yaw_plot = zeros(1,sim_length-2);

    des_roll_plot = zeros(1,sim_length-2);
    des_pitch_plot = zeros(1,sim_length-2);
    des_yaw_plot = zeros(1,sim_length-2);

    z_plot = zeros(1,sim_length-2);
    x_plot = zeros(1,sim_length-2);
    y_plot = zeros(1,sim_length-2);
    x_error_plot = zeros(1,sim_length-2);
    y_error_plot = zeros(1,sim_length-2);
    z_error_plot = zeros(1,sim_length-2);

    % Store the data at each time steps
    state_v = repmat(state, 1, sim_length-1);
    error_p = repmat(pos_err, 1, sim_length-1);
    error_a = repmat(att_err, 1, sim_length-1);
    u_tt = [];
    state_v(1) = state;
    

    desiredAngles.theta_d = 0;
    desiredAngles.phi_d = 0;
    desiredAngles.psi_d = 0;
    
    %% MAIN LOOP
    for i = 1:TT*ATTITUTDE_LOOP_RATE - 1
        if (mod(i,ceil(ATTITUTDE_LOOP_RATE/POSITION_LOOP_RATE) ) == 0 || i == 1)
            desired_pos = [des.pos(start_t + i,:), des.yaw(start_t + i)];
            % Switch depedning on the decision variable logic_q
            if logic_q == 0
                [T, theta_d, phi_d, p_er] = position_controller(pos_err, state_v(i), desired_pos, dt_pos, params);
            else
                [T, theta_d, phi_d, p_er] = position_controller2(pos_err, state_v(i), desired_pos, dt_pos, params);
            end
        end
        % Store the position Errors
        error_p(i) = p_er;
        
        desiredAngles.theta_d = theta_d;
        desiredAngles.phi_d = phi_d;
        desiredAngles.psi_d = des.yaw(start_t + i);

        % Compute the Moments needed to be applied to the quarotor for the current state.
        input = [T; theta_d; phi_d; des.yaw(start_t + i)];
        [st_v, u_t, CL_at_err] = DroneClosedLoopModel(att_err, state_v(i), input, dt, params, 0);
        error_a(i) = CL_at_err;
        
        % Store the state and computed inputs for the current time step.
        state_v(i+1) = st_v;
        u_tt = [u_tt; u_t];

         %% Saving data for plotting purposes
         x_plot(i) = st_v.x;
         y_plot(i) = st_v.y; 
         z_plot(i) = st_v.z;

         roll_plot(i) = st_v.phi;
         pitch_plot(i) = st_v.theta;
         yaw_plot(i) = st_v.psi;

         x_error_plot(i) = st_v.x - des.pos(start_t + i,1);
         y_error_plot(i) = st_v.y - des.pos(start_t + i,2);
         z_error_plot(i) = st_v.z - des.pos(start_t + i,3);
         
         des_roll_plot(i) = desiredAngles.phi_d;
         des_pitch_plot(i) = desiredAngles.theta_d;
         des_yaw_plot(i) = desiredAngles.psi_d;
    end

end