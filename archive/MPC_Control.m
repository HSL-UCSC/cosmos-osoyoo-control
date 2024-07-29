% MPC controller

% model predictive controller parameters, takes in position and heading
% x, y, theta (current_heading), target_x, target_y, target_theta (angle between current_pos and target_pos)
% returns v (velocity) and steering angle (gamma)

classdef MPC_Control
    properties
        C;  % control horizon length
        N;  % prediction horizon length
        dt; % time step
        Q; % state cost matrix
        R; % control cost matrix
        
        systemModel = bicycleKinematics("WheelBase", 0.162, "VehicleSpeedRange", [0, 0.1], "MaxSteeringAngle", deg2rad(40), "VehicleInputs", "VehicleSpeedSteeringAngle");
        
        x = 0;
        y = 0;
        theta = 0;
        x_target = 0;
        y_target = 0;
        theta_target = 0;
        v = 0;
        gamma = 0;
        prediction;
        control;
        control_index;
        max_gamma = deg2rad(40);
    end
    
    methods
        function obj = MPC_Control(dt)
            obj.C = 5;
            obj.N = 20; % prediction horizon length
            obj.dt = dt; % time step
%             obj.Q = [1/0.1 0; 0 0.5/deg2rad(40)]; % relative weights of distance and heading error
%             obj.R = [0.25/0.1 0; 0 0.25/deg2rad(40)]; % input cost matrix
            obj.Q = [1/0.1; 0.5/obj.max_gamma];
            obj.R = [0.25/0.1; 0.25/obj.max_gamma];
            obj.control_index = obj.C;
        end
        
        function obj = update(obj, x, y, theta, x_target, y_target, theta_target)
            % x, y, theta: current state
            % x_target, y_target, theta_target: target state
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            obj.x_target = x_target;
            obj.y_target = y_target;
            obj.theta_target = theta_target;
        end
        
        function [v, gamma, obj] = get_control(obj)
            % get_control: computes the optimal control given the current state and target state
            % returns v (velocity) and gamma (steering angle)
            if (obj.control_index == obj.C)
                u0 = zeros(obj.N, 2);
                u0(1, :) = [obj.v, obj.gamma];
                obj.prediction = zeros(obj.N-1, 3);
                u_N = fmincon(@(u_N)obj.cost(u_N), u0, [], [], [], [], repmat([0.05 -obj.max_gamma], obj.N, 1), repmat([0.1 obj.max_gamma], obj.N, 1));
                
                % fill out obj.prediction using derivative(systemModel) and u_N
                obj.prediction(1, :) = [obj.x, obj.y, obj.theta];
                for i = 2:obj.N
                    state = obj.prediction(i - 1, :);
                    u_i = u_N(i - 1, :);
                    state_delta = derivative(obj.systemModel, state, u_i);
                    obj.prediction(i, :) = state + state_delta' * obj.dt;
                end
                obj.control = u_N;
                disp(u_N);
                disp(cost(obj, u_N));
                obj.control_index = 1;
            end
            v = obj.control(obj.control_index, 1);
            gamma = obj.control(obj.control_index, 2);
            obj.control_index = obj.control_index + 1;
        end
        
        function cost = cost(obj, u)
            % cost: computes the cost of a given control
            % returns cost
            % u: control
            state = [obj.x, obj.y, obj.theta];
            cost = 0;
            for i = 1:obj.N
                error_d = abs(hypot(obj.x_target - state(1), obj.y_target - state(2)));
                error_theta = abs(obj.theta_target - state(3));
                error = [error_d, error_theta];

                u_i = u(i, :);
                u_i_abs = abs(u_i);
%                 cost = cost + error * obj.Q * error' + u_i_abs * obj.R * u_i_abs';
                cost = cost + error * obj.Q + u_i_abs * obj.R;
                % update error using obj.systemModel
                state_delta = derivative(obj.systemModel, state, u_i);
                state = state + state_delta' * obj.dt;
            end
        end
        
        function [out, obj] = true(obj)
            out = false;
        end
    end
end