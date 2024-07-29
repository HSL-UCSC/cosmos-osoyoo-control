classdef Dog
    properties
        dllPath = fullfile('d:','StDroneControl','NatNetSDK','lib','x64','NatNetML.dll');
        HostIP = '127.0.0.1';
        Dog_ID = 1; % Rigid body ID of the dog from Motive
        udp = udpport; % intializing udp protocol
        theClient = nan;
        assemblyInfo = nan;
        x = -Inf;
        y = -Inf;
        theta;
        max_delta_theta = pi/16;
        
        % IP address & port of Ubuntu VM used to communicate w/ dog
        vm_ip = '192.168.254.134'
        vm_port = 1145

        command = zeros(1, 11, 'single') % command to be sent to vm/dog

        theta_control; % true if theta control is enabled
        init_time = 0; % time at which the dog was initialized
        stop_time = 0; % time at which the dog was stopped
        
        buf_size = 100;
        values;
        index = 1;
        buf_init_size = 0;
        filter_out = 0;
    end
    methods
        function obj = Dog(theta_control)
            obj.theta_control = theta_control;

            % Connect to Motive
            obj.assemblyInfo = NET.addAssembly(obj.dllPath); % Add API function calls
            obj.theClient = NatNetML.NatNetClientML(0);
            % Create connection to localhost, data is now being streamed through client object
            obj.theClient.Initialize(obj.HostIP, obj.HostIP);
        end
        function obj = drive(obj, v, gamma, ~)
            obj.command(1) = 2;
            obj.command(10) = 0; % side to side
            obj.command(9) = v * 5; % forward/backward
            obj.command(11) = gamma; % rotation

            % Create a UDP object
            Robot_Dog_UDP = udpport("LocalHost",'192.168.254.1');
            % Transform single float to uint8
            Control_Data_Uint8 = typecast(obj.command,'uint8');
            % Send Command
            write(Robot_Dog_UDP, Control_Data_Uint8, obj.vm_ip, obj.vm_port);
        end
        function [x, y, theta, obj] = odom(obj)
            % Retreving current position of car
%             [time, x, ~, y, ~, ~, theta] = 
            result = GetDronePosition(obj.theClient, obj.Dog_ID);
            time = result(1);
            
            x = double(result(2));
            y = double(result(3));
            theta = double(mod(result(7) - pi/2, 2 * pi));

            if time ~= 0
                obj.init_time = time;
            end
            
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
        end
        function obj = weighted_low_pass_filter(obj, input)
            obj.values(obj.index) = input;
            obj.index = mod(obj.index, obj.buf_size)+1;

            obj.buf_init_size = min([obj.buf_init_size + 1, obj.buf_size]);
            
            obj.filter_out = mean(obj.values(1:obj.buf_init_size));
        end
    end
end