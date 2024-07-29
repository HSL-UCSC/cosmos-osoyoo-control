classdef CustomPurePursuit_Control
    properties
        x = 0;
        y = 0;
        theta = 0;
        v = 0; % Linear velocity
        gamma = 0; % Steering angle
        prediction = [];
        controller; % Instance of CustomPurePursuit
        waypoints;
        lastX = 0; % Previous X position
        lastY = 0; % Previous Y position
        DT = 0.4; % Fixed time step for the simulation
        reached_path = false;
        first_point = [];
        dist_threshold;
    end
    
    methods
        function obj = CustomPurePursuit_Control(waypoints, lookahead_dist, dist_threshold)
            % Initialize the custom pure pursuit controller with the desired parameters
            obj.controller = CustomPurePursuit('Waypoints', waypoints, ...
                                               'DesiredLinearVelocity', 0.1, ...
                                               'MaxAngularVelocity', 2*deg2rad(40), ...
                                               'LookaheadDistance', lookahead_dist);
            obj.waypoints = waypoints;
            obj.dist_threshold = dist_threshold;
        end
        
        function obj = update(obj, x, y, theta, x_target, y_target, ~)
            % Update the current state and target for the controller
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            
            % Calculate speed based on the change in position over the fixed time step, DT
            dx = obj.x - obj.lastX;
            dy = obj.y - obj.lastY;
            speed = sqrt(dx^2 + dy^2) / obj.DT; % Speed is the magnitude of the velocity vector
            
            % Update last positions for the next speed calculation
            obj.lastX = x;
            obj.lastY = y;
            
            % Update the waypoints for the controller
%             obj.controller.Waypoints = [x_target, y_target]; % Set new target as waypoints
            
            % Explicitly call interpolateWaypoints if necessary
%             obj.controller = obj.controller.interpolateWaypoints(); % This line ensures waypoints are interpolated
            
            if isempty(obj.first_point)
                obj.first_point = [x_target y_target];
                obj.controller.Waypoints = [x_target y_target];
                obj.controller = obj.controller.interpolateWaypoints();
            elseif ~obj.reached_path && ~isequal(obj.first_point, [x_target y_target])
                obj.reached_path = true;
                obj.controller.Waypoints = obj.waypoints;
                obj.controller = obj.controller.interpolateWaypoints();
            end

            % Compute the control commands using the updated state and calculated speed
            [obj.v, obj.gamma, obj.controller] = obj.controller.control([x, y, theta], speed);
            
            % Compute and store the prediction or lookahead point for visualization or logging
            [~, lookaheadPoint, obj.controller] = obj.controller.findLookAheadPoint([x, y, theta]);
            obj.prediction = [obj.prediction; lookaheadPoint];

            disp("v gamma")
            disp(obj.v)
            disp(obj.gamma);
        end
        
        function [v, gamma, obj] = get_control(obj)
            % Return the computed velocity and steering angle
            v = obj.v;
            gamma = obj.gamma;
        end

        function [out, obj] = done(obj)
            out = (obj.controller.WaypointIndex == length(obj.controller.InterpolatedWaypoints)) && ...
                (abs(norm([obj.x, obj.y] - obj.controller.InterpolatedWaypoints(obj.controller.WaypointIndex, :))) < obj.dist_threshold);
        end
    end
end
