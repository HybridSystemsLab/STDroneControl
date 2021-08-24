function [cost] = EvaluateCost(X_r, Y_r, Z_r, U)
    %% Input Arguments
    % X_r: An array containg the error in state x for the prediction horizon
    % Y_r: An array containg the error in state y for the prediction horizon
    % Z_r: An array containg the error in state z for the prediction horizon
    % U: An array of 4x1 Input vector consisting of Thrust and Mometns ([T; M])for the prediction horizon

    
    %% Ouput Argument
    % cost: Evaluted cost for the set of inputs.

    % X cost
    x_w = 1.2;
    x_cost = norm(X_r)^2;
    
    % Y cost
    y_w = 0.5;
    y_cost = norm(Y_r)^2;
    
    % Z cost
    z_w = 1.0;
    z_cost = norm(Z_r)^2;
    
    % XY cost
    xy_w = 0.1;
    xy_cost = norm(X_r)^2 + norm(Y_r)^2;

    % XYZ cost
    %     xyz_w = 5;
    %     xyz_cost = xy_cost + z_cost;
    
    % Input cost
    u_w = [0.001 0.001 0.001 0.001];
    S_u = sqrt(diag(U'*U));
    input_cost = u_w*S_u;
    
    %% TOTAL COST
    cost = x_w*x_cost + y_w*y_cost + z_w*z_cost + xy_w*xy_cost + input_cost;
%     cost = z_w*z_cost + xy_w*xy_cost + xyz_w*xyz_cost;
end

