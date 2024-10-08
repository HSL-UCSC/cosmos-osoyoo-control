classdef Basic_Control
    properties
        max_v;
        max_gamma;
        x;
        y;
        theta;
        x_target;
        y_target;
        theta_target;
        % wheel_base = 11; % Distance between the wheels
    end
    methods
        function obj = Basic_Control(max_v, max_gamma)
            obj.max_v = max_v;
            obj.max_gamma = max_gamma;
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
            v = min(obj.max_v, distance);
            gamma = mod(obj.theta_target - obj.theta, 2 * pi);
            if gamma > pi
                gamma = gamma - 2 * pi;
            end
            gamma = min(obj.max_gamma, max(-obj.max_gamma, gamma));
            % v = v + obj.max_v / 3 * abs(gamma / obj.max_gamma);

            % v = min(v, (1 - ((abs(gamma) / obj.max_gamma) / 4)) * obj.max_v);

            % distance
            % v
            % gamma
        end

        function [out, obj] = done(obj)
            out = false;
        end
    end
end