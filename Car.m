classdef Car
    properties
        dllPath = fullfile('d:','StDroneControl','NatNetSDK','lib','x64','NatNetML.dll');
        HostIP = '127.0.0.1';
        Car_ID = 1; % Rigid body ID of the car from Motive
        udp = udpport; % intializing udp protocol
        theClient = nan;
        assemblyInfo = nan;
        x = -Inf;
        y = -Inf;
        theta;
        v;
        gamma;
        max_delta_theta = pi/16;
        wheel_base = 0.2;
        max_v = 0.1;
        buf_size = 100;
        values;
        index = 1;
        buf_init_size = 0;
        filter_out = 0;
    end
    methods
        function obj = Car()
            obj.assemblyInfo = NET.addAssembly(obj.dllPath); % Add API function calls
            obj.theClient = NatNetML.NatNetClientML(0);
            % Create connection to localhost, data is now being streamed through client object
            obj.theClient.Initialize(obj.HostIP, obj.HostIP);
            
            obj.values = zeros(1, obj.buf_size);
        end
        
        function obj = drive(obj, v, gamma, ~)
            %             disp(v);
            %             disp(gamma);
            obj.v = v;
            obj.gamma = gamma;
            if (v > 0)
                v = min([85 + v*10*(255-85), 255]);
            else
                v = 0;
            end
            gamma = round(1 * rad2deg(gamma));
            % gamma = gamma + 40;
            
            % send command to car
            Lspeed = v - (gamma * obj.wheel_base / 2);
            Rspeed = v + (gamma * obj.wheel_base / 2);
            
            % Scale speeds to uint8 range (0-255)
            Lspeed = (max(min(Lspeed * 255 / obj.max_v, 255), 0));
            Rspeed = (max(min(Rspeed * 255 / obj.max_v, 255), 0));
            write(obj.udp, "L"+Lspeed, 'uint8', '192.168.0.161', 8888);
            write(obj.udp, "R"+Rspeed, 'uint8', '192.168.0.161', 8888);
        end
        
        function [x, y, theta, obj] = odom(obj)
            % Retreving current position of car
            [CarPos] = GetDronePosition(obj.theClient, obj.Car_ID);
            
            x = double(CarPos(2)); % Taking x point of the car
            y = double(CarPos(3)); % Taking y point of the car
            theta = double(mod(CarPos(7) - pi/2, 2 * pi)); % taking current heading angle of car
            
            % motive returns an inverted theta under certain conditions
            if (y > obj.y)
                theta = theta * -1;
            end
            %             if (abs(theta - obj.theta) > obj.max_delta_theta)
            %                 theta = atan2(y - obj.y, x - obj.x);
            %             end
            
            
            % keep track of car's position
            if (abs(x - obj.x) > 0.01)
                obj.x = x;
            end
            if (abs(y - obj.y) > 0.01)
                obj.y = y;
            end
            
            obj = obj.weighted_low_pass_filter(theta);
            obj.theta = obj.filter_out;
            %             disp(obj.theta);
        end
        
        function obj = weighted_low_pass_filter(obj, input)
            obj.values(obj.index) = input;
            obj.index = mod(obj.index, obj.buf_size)+1;
            %             if obj.buf_init_size < obj.buf_size
            %                 obj.buf_init_size = obj.buf_init_size + 1;
            %             end
            obj.buf_init_size = min([obj.buf_init_size + 1, obj.buf_size]);
            
            obj.filter_out = mean(obj.values(1:obj.buf_init_size));
            %             disp(obj.filter_out);
            
            %             disp(obj.values)
            disp(obj.index)
            %             disp(circshift(1:obj.buf_init_size, obj.index - 1))
            % %             obj.filter_out = mean(circshift(1:obj.buf_init_size, obj.index - 1).*obj.values(1:obj.buf_init_size)) * 2 / (obj.buf_init_size * (obj.buf_init_size - 1));
            %             disp(obj.filter_out)
        end
    end
end