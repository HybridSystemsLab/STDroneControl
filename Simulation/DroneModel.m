function [state] = DroneModel(state, input, dt, params)
    %% Input Arguments
    % state: A struct containg the states of the quadrotor/drone [x; y; z; vx; vy; vz; phi; theta; psi; w_x; w_y; w_z];
    % input: 4x1 Input vector consisting of Thrust and Mometns ([T; M]).
    % dt: sampling time
    % params: A struct containing the parameters of the quadrotor/drone.
    
    %% Ouput Argument
    % state: A struct containg the updated states of the quadrotor/drone after applying the input to the dynamics.


    g = params.g ;
    m = params.m ;

    Ix = params.Ix;
    Iy = params.Iy;
    Iz = params.Iz;

    phi = state.phi;
    theta = state.theta;
    psi = state.psi;
    w_x = state.w_x; 
    w_y = state.w_y;
    w_z = state.w_z;
    x = state.x;
    y = state.y;
    z = state.z;
    vx = state.vx;
    vy = state.vy;
    vz = state.vz;

    up = input(2);
    uq = input(3);
    ur = input(4);
    thrust = input(1);

    % ------------------- Update the Rotational Dynamics -------------------


    theta_new = theta + dt*(w_x + tan(phi)*(w_y*sin(theta) - w_z*cos(theta)));    
    phi_new = phi + dt*(w_y*cos(theta) + w_z*sin(theta));
    psi_new = psi + (dt* ( ( w_z*cos(theta) - w_y*sin(theta) ) /cos(phi) ) );  

    w_x_new = w_x + (dt*(up - Iy*w_y*w_z + Iz*w_y*w_z)/Ix);
    w_y_new = w_y + (dt*(uq + Ix*w_x*w_z - Iz*w_x*w_z)/Iy);
    w_z_new = w_z + (dt*(ur - Ix*w_x*w_y + Iy*w_x*w_y)/Iz);

    % ------------------- Update the Translational Dynamics -------------------
    x_new = x + dt*(vx);
    y_new = y + dt*(vy);
    z_new = z + dt*(vz);

    vx_new = vx + dt*(thrust/m * (sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi)));
    vy_new = vy + dt*(thrust/m * (cos(theta)*sin(phi)*sin(psi) - cos(psi)*sin(theta)));
    vz_new = vz + dt*(thrust/m * (cos(phi)*cos(theta)) - g);

    % ----------- Store the updated states back into the struct -----------
    state.phi = phi_new;
    state.theta = theta_new;
    state.psi = psi_new;
    state.w_x = w_x_new;
    state.w_y = w_y_new;
    state.w_z = w_z_new;
    state.x = x_new;
    state.y = y_new;
    state.z = z_new;
    state.vx = vx_new;
    state.vy = vy_new;
    state.vz = vz_new;

end