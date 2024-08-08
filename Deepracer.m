classdef Deepracer
    properties
        Car_ID = "Object1";
        client;
        
        udp = udpport; % intializing udp protocol
        theClient = nan;
        assemblyInfo = nan;
        x = 0;
        y = 0;
        theta = 0;
        v;
        gamma;
        max_delta_theta = pi/16;
        wheel_base = 700;
        max_v = 1000;
        max_gamma = deg2rad(30);
        buf_size = 100;
        values;
        index = 1;
        buf_init_size = 0;
        filter_out = 0;
    end
    methods
        function obj = Deepracer()
            % obj.client = Vicon.Client();
            % obj.client.destroy();
            % obj.client.initialize();
            
            obj.values = zeros(1, obj.buf_size);
        end
        
        function obj = drive(obj, v, gamma, ~)
            obj.v = v;
            obj.gamma = gamma;

            v = v * 1.0 / obj.max_v;
            gamma = gamma * 1.0 / obj.max_gamma;

            v = typecast(v, 'uint8');
            gamma = typecast(gamma, 'uint8');
            
            % send command to car
            write(obj.udp, [v gamma], 'uint8', '128.114.59.181', 8888);
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