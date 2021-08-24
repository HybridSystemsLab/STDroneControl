function state = init_state()
    %% Input Arguments
    % NONE
    
    %% Ouput Argument
    % state: A struct containing the state of the quadrotor/drone
    
    % States: 
    %      - (x,y,z): position of the quadrotor/drone
    %      - (vx,vy,vz): linear velocity of the quadrotor/drone
    %      - (theta,phi,psi): Euler angles of the quadrotor/drone
    %      - (w_x,w_y,w_z): angular velocities of the quadrotor/drone
    
    % Initializing States
    x0 = 0;
    y0 = 0;
    z0 = 0;
  
    state.x = x0;
    state.y = y0;
    state.z = z0;


    state.phi = deg2rad(0);
    state.theta = deg2rad(0);
    state.psi = deg2rad(0);

    state.w_x = 0;
    state.w_y = 0;
    state.w_z = 0;

    state.vx = 0;
    state.vy = 0;
    state.vz = 0;
end