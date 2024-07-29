classdef Basic_Control
    properties
        max_v = 0.1;
        max_gamma = deg2rad(40);
        x;
        y;
        theta;
        x_target;
        y_target;
        theta_target;
        wheel_base = 0.2; % Distance between the wheels
    end
    methods
        function obj = Basic_Control()
        end

        function obj = update(obj, x, y, theta, x_target, y_target, theta_target)
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            obj.x_target = x_target;
            obj.y_target = y_target;
            obj.theta_target = theta_target;
        end

        function [v, gamma, obj] = get_control(obj)
            distance = hypot(obj.x_target - obj.x, obj.y_target - obj.y);
            v = min(obj.max_v * 2 / 3, distance);
            gamma = mod(obj.theta_target - obj.theta, 2 * pi);
            if gamma > pi
                gamma = gamma - 2 * pi;
            end
            v = v + obj.max_v / 3 * abs(gamma / obj.max_gamma);
        end

        function control_car(obj)
            [v, gamma, ~] = obj.get_control();
            Lspeed = v - (gamma * obj.wheel_base / 2);
            Rspeed = v + (gamma * obj.wheel_base / 2);

            % Scale speeds to uint8 range (0-255)
            Lspeed = (max(min(Lspeed * 255 / obj.max_v, 255), 0));
            Rspeed = (max(min(Rspeed * 255 / obj.max_v, 255), 0));
            clear;
            clc;
            udp = udpport;

            % Send the commands via UDP
            write(udp, "L"+100, 'uint8', '192.168.0.161', 8888);
            write(udp, "R"+100, 'uint8', '192.168.0.161', 8888);
        end

        function [out, obj] = true(obj)
            out = false;
        end
    end
end