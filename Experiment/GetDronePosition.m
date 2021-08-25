function [result] = GetDronePosition(client,id)
    %% Function Purpose: 
    % This function will get the data from Motive and extract the x, y, z
    % coordinates from the incoming data.
    
    %% Input:
    % client: The motive client object responsible for communicating with motive
    % id : The ridid body id for which the data needs to be extracted
    
    %% Output: 
    % result: 1x7 vector containg the timestamp, x, y, z, pitch, roll, and yaw of the rigid body.

    % Get the frame data
    frameData = client.GetLastFrameOfData();

    %Get the time stamp
    time_stamp = frameData.fTimestamp;

    %Get the marker data
    drone_pos = frameData.RigidBodies(id);

    if length(drone_pos) > 0

        %% Motive coordiate frame
        %        ^ z
        %        |
        %        |
        % x <----O y(pointing up)
        %

        %% Our coordiate frame
        % 
        % x <----O z(pointing up)
        %        |
        %        |
        %        v y

        x_d = drone_pos.x;
        y_d = -drone_pos.z; 
        z_d = drone_pos.y;
        q = [drone_pos.qx, drone_pos.qy, drone_pos.qz, drone_pos.qw];
        Eul_ang = quat2eul(q);
        yaw = -Eul_ang(2);
        if Eul_ang(1) <= 0
            pitch = pi + Eul_ang(1);
        else
            pitch = Eul_ang(1)-pi;
        end

        roll = Eul_ang(3);
    else
        x_d = 0;
        y_d = 0;
        z_d = 0;
        yaw = 0;
    end

    result = [time_stamp, x_d, y_d, z_d, pitch, roll, yaw];

end


