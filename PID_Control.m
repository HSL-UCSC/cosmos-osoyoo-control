% PID Control
% consists of PID controller for position and heading

classdef PID_Control
    properties
        % PID parameters
        Kp_pos = 0.5;
        Ki_pos = 0.0;
        Kd_pos = 0.0;
        Kp_head = 0.5;
        Ki_head = 0.0;
        Kd_head = 0.0;

        % PID objects
        pid_pos;
        pid_head;

        % limits
        max_pos = 0.5;
        max_head = 45 * pi / 180;

    end

    methods
        % constructor
        function obj = PID_Control()
            obj.pid_pos = PID(obj.Kp_pos, obj.Ki_pos, obj.Kd_pos, obj.max_pos);
            obj.pid_head = PID(obj.Kp_head, obj.Ki_head, obj.Kd_head, obj.max_head);
        end

        function obj = update(obj, x, y, theta, x_target, y_target, theta_target)
            % update PID parameters
            obj.pid_pos = obj.pid_pos.update(hypot(x_target - x, y_target - y), 0.05);
            obj.pid_head = obj.pid_head.update(mod(theta_target - theta, 2*pi), 0.05);
        end

        function [v, w, obj] = get_control(obj)
            % get control
            v = obj.pid_pos.get_control();
            w = obj.pid_head.get_control();
            if (w > pi)
                w = w - 2 * pi;
            end
        end

        function [out, obj] = true(obj)
            out = false;
        end
    end
end