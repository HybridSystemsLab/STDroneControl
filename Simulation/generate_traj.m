function desired_traj = generate_traj(num, T, dt)
    %% Input Arguments
    % num : variabel to choose between circular(1) and 8 figure(2) trajectory.
    % T: How long do you want to generate the trajectory for
    % dt: sampling time
    
    %% Ouput Argument
    % desired_traj: A struct containing the desired trajectory (pos, vel, acc, etc.)

    time = 0:dt:T;
    if num == 1 % Generate an circular trajectory.

        p_d = [cos(0.1*time)',sin(0.1*time)',(1 + 0*cos(time))']; % desired x y z
        psi_d = 0*sin(2.5*time)'; % desired yaw

        psi_dot = diff(psi_d)/dt;

        v_d = [diff(p_d(:,1))/dt,diff(p_d(:,2))/dt,diff(p_d(:,3))/dt]; % desired velocity

        a_d = [diff(v_d(:,1))/dt,diff(v_d(:,2))/dt,diff(v_d(:,3))/dt]; % desired acceleration

        desired_traj.pos = p_d;
        desired_traj.velocity = v_d;
        desired_traj.yaw = psi_d;
        desired_traj.yaw_rate = psi_dot;
        desired_traj.acc = a_d;
    %end
    
    % Generate an 8 figure.
    elseif num == 2
        sim_length = length(time);
        p_d_x = zeros(1,sim_length-2);
        p_d_y = zeros(1,sim_length-2);
        p_d_z = zeros(1,sim_length-2);
        
        for i = 1:length(time)
            p_d_x(i) = 0.5*cos(0.1*time(i))/(1 + (sin(0.1*time(i)))^2);
            p_d_y(i) = 0.5*sin(0.1*time(i))*cos(0.1*time(i))/(1 + (sin(0.1*time(i)))^2);
            p_d_z(i) = (1 + 0*cos(0.1*time(i)))';
        end
        
        p_d = [p_d_x', p_d_y', p_d_z'];% desired x y z 
        psi_d = 0*sin(2.5*time)'; % desired yaw

        psi_dot = diff(psi_d)/dt;

        v_d = [diff(p_d(:,1))/dt,diff(p_d(:,2))/dt,diff(p_d(:,3))/dt]; % desired velocity

        a_d = [diff(v_d(:,1))/dt,diff(v_d(:,2))/dt,diff(v_d(:,3))/dt]; % desired acceleration

        desired_traj.pos = p_d;
        desired_traj.velocity = v_d;
        desired_traj.yaw = psi_d;
        desired_traj.yaw_rate = psi_dot;
        desired_traj.acc = a_d;
     end
end