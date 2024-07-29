function [result] = GetDronePosition(client,id)
% This function will get the data from Motive and extract the x, y, z
% coordinates from the incoming data.

frameData = client.GetLastFrameOfData();

%Get the time stamp
time_stamp = frameData.fTimestamp;

%Get the marker data
drone_pos = frameData.RigidBodies(id);

if ~isempty(drone_pos)
    
    %% Motive coordiate frame
    %        ^ z
    %        |
    %        |
    % x <----O y(pointing up)
    %

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
    pitch = 0;
    roll = 0;
    yaw = 0;
end

result = [time_stamp, x_d, y_d, z_d, pitch, roll, yaw];

end


