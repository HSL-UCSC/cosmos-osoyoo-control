% MPC controller

% model predictive controller parameters, takes in position and heading
% x, y, theta (current_heading), target_x, target_y, target_theta (angle between current_pos and target_pos)
% returns v (velocity) and steering angle (gamma)

classdef PurePursuit_Control
    properties
        x = 0;
        y = 0;
        theta = 0;
        v = 0;
        gamma = 0;
        prediction;
        controller;
        waypoints;
    end
    
    methods
        function obj = PurePursuit_Control(waypoints, lookahead_dist, max_v, max_gamma)
            obj.waypoints = waypoints;
            obj.controller = controllerPurePursuit('DesiredLinearVelocity', max_v, 'MaxAngularVelocity', max_gamma, 'LookaheadDistance', lookahead_dist);
        end
        
        function obj = update(obj, x, y, theta, x_target, y_target, ~)
            % x, y, theta: current state
            % x_target, y_target, theta_target: target state
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            obj.controller.Waypoints = [x_target y_target];
            [obj.v, obj.gamma, lookaheadPoint] = obj.controller([x, y, theta])
            obj.prediction = [obj.prediction; lookaheadPoint];
        end
        
        function [v, gamma, obj] = get_control(obj)
            v = obj.v;
            gamma = obj.gamma;
        end

        function [out, obj] = done(obj)
            out = false;
        end
        
    end
end