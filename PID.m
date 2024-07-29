% PID controller 

% PID controller parameters, takes in position and heading
% x, y, theta (current_heading), target_x, target_y, target_theta (angle between current_pos and target_pos)
% returns a vector of the control inputs
classdef PID
    properties
        % PID gains
        Kp % proportional gain
        Ki % integral gain
        Kd % derivative gain
        
        % PID state
        error % current error
        error_prev % previous error
        error_sum % sum of errors
        error_diff % difference of errors
        error_int % integral of errors
        
        % PID limits
        limit % limit of control input
    end

    methods
        % constructor
        function obj = PID(Kp, Ki, Kd, limit)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.limit = limit;
            obj.error = 0;
            obj.error_prev = 0;
            obj.error_sum = 0;
            obj.error_diff = 0;
            obj.error_int = 0;
        end
        
        % update PID state
        function obj = update(obj, error, dt)
            tmp = obj.error;
            obj.error = error;
            obj.error_diff = (obj.error - obj.error_prev) / dt;
            obj.error_sum = obj.error_sum + obj.error * dt;
            obj.error_int = obj.error_sum;
            obj.error_prev = tmp;
        end
        
        % get control input
        function u = get_control(obj)
            u = obj.Kp * obj.error + obj.Ki * obj.error_int + obj.Kd * obj.error_diff;
            u = min(max(u, -obj.limit), obj.limit);
        end
    end
end