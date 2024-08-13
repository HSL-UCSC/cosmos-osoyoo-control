classdef OsoyooV2
    properties
        Car_ID = "Object1";
        client;
        
        udp = udpport; % intializing udp protocol
        theClient = nan;
        assemblyInfo = nan;
        x = -Inf;
        y = -Inf;
        theta;
        v;
        gamma;
        max_delta_theta = pi/16;
        wheel_base = 700;
        max_v = 1000;
        buf_size = 100;
        values;
        index = 1;
        buf_init_size = 0;
        filter_out = 0;
    end
    methods
        function obj = OsoyooV2()
            obj.client = Vicon.Client();
            obj.client.destroy();
            obj.client.initialize();
            
            obj.values = zeros(1, obj.buf_size);
        end
        
        function obj = drive(obj, v, gamma, ~)
            obj.v = v;
            obj.gamma = gamma;
            
            % send command to car
            Lspeed = v - (gamma * obj.wheel_base / 2);
            Rspeed = v + (gamma * obj.wheel_base / 2);
            
            % Scale speeds to uint8 range (0-255)
            Lspeed = round(max(min(Lspeed * 255 / obj.max_v, 255), 0));
            Rspeed = round(max(min(Rspeed * 255 / obj.max_v, 255), 0));
            write(obj.udp, "L"+Lspeed, 'uint8', '192.168.0.161', 8888);
            write(obj.udp, "R"+Rspeed, 'uint8', '192.168.0.161', 8888);
        end
        
        function [x, y, theta, obj] = odom(obj)
            pose = obj.client.get_pose(obj.Car_ID, obj.Car_ID);
            % these might be switched up
            obj.x = double(pose.translation{1});
            obj.y = double(pose.translation{2});
            obj.theta = mod(double(pose.rotation{3}) - pi/2, 2*pi); % if car is drifting in wrong direction, add a 90 degree offset +/-

            if (obj.theta > pi)
                obj.theta = obj.theta - 2*pi;
            end
            
            x = obj.x;
            y = obj.y;
            theta = obj.theta;
        end
        
        function obj = weighted_low_pass_filter(obj, input)
            obj.values(obj.index) = input;
            obj.index = mod(obj.index, obj.buf_size)+1;
            obj.buf_init_size = min([obj.buf_init_size + 1, obj.buf_size]);
            
            obj.filter_out = mean(obj.values(1:obj.buf_init_size));
            disp(obj.index)
        end
    end
end