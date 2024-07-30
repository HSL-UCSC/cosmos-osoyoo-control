classdef Basic_Control
    properties
        max_v = 1000;
        max_gamma = deg2rad(90);
        x;
        y;
        theta;
        x_target;
        y_target;
        theta_target;
        % wheel_base = 11; % Distance between the wheels
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
            % v = v + obj.max_v / 3 * abs(gamma / obj.max_gamma);

            % distance
            % v
            % gamma
        end

        function [out, obj] = done(obj)
            out = false;
        end
    end
end