function params = drone_params()
    %% Input Arguments
    % NONE
    
    %% Ouput Argument
    % params: A struct containing all the parameters of the ST Drone

    m = 0.052; % kg   Mass
    g = 9.81; % m/s/s Gravity


    params.m = m;
    params.g = g;

    % In kg-m^2   Inertia Tensor
    params.Ix = 0.000081061184; 
    params.Iy = 0.000081061184; %0.000081939103; %0.28
    params.Iz = 0.0001594886; %0.545;

    params.L = 0.1; % in meters Arm Length

    % Maximum angles allowed
    params.maxPhi = pi/6;
    params.minPhi = -pi/6;

    params.maxTheta = pi/6;
    params.minTheta = -pi/6;

end

