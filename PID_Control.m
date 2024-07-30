% PID Control
% consists of PID controller for position and heading

classdef PID_Control
    properties
        %%%%%%% BEGIN CODE %%%%%%%

        % PID parameters
        Kp_pos = 200;
        Ki_pos = 10.0;
        Kd_pos = 0.0;
        Kp_head = 200;
        Ki_head = 0.0;
        Kd_head = 0.0;

        %%%%%%% END CODE %%%%%%%

        % PID objects
        pid_pos;
        pid_head;

        % limits
        max_v = 1000;
        max_steering = deg2rad(180);

    end

    methods
        % constructor
        function obj = PID_Control()
            obj.pid_pos = PID(obj.Kp_pos, obj.Ki_pos, obj.Kd_pos, obj.max_v);
            obj.pid_head = PID(obj.Kp_head, obj.Ki_head, obj.Kd_head, obj.max_steering);
        end

        function obj = update(obj, x, y, theta, x_target, y_target, theta_target)
            % update PID parameters
            obj.pid_pos = obj.pid_pos.update(hypot(x_target - x, y_target - y), 0.05);
            theta_diff = mod(theta_target - theta, 2*pi);
            if theta_diff > pi
                theta_diff = theta_diff - 2*pi;
            end
            obj.pid_head = obj.pid_head.update(theta_diff, 0.05);
        end

        function [v, w, obj] = get_control(obj)
            % get control
            v = obj.pid_pos.get_control();
            w = obj.pid_head.get_control();
            if (w > pi)
                w = w - 2 * pi;
            end
        end

        function [out, obj] = done(obj)
            out = false;
        end
    end
end