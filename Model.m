%% system model v3 using bicycle model
% using https://www.mathworks.com/help/nav/ref/bicyclekinematics.html

classdef Model
    properties
        wheel_base = 700;
        max_v = 1000;
        max_gamma = deg2rad(90);
    
        % bicycle model
        systemModel; % = bicycleKinematics("WheelBase", wheel_base, "VehicleSpeedRange", [-max_v, max_v], "MaxSteeringAngle", pi, "VehicleInputs", "VehicleSpeedSteeringAngle");
        % initialState will be 0, 0, 0 for simulated model
        initialState = [0, 0, 0];
        % simulated input for x, y
        x = 0;
        y = 0;
        % linear velocity
        v = 0;
        % heading angle
        theta = -pi/4;
        % steering angle
        gamma = 0;
    end
    
    methods
        function obj = Model()
            %%% CHANGE MODEL HERE %%%
            obj.systemModel = bicycleKinematics("WheelBase", obj.wheel_base, "VehicleSpeedRange", [0, obj.max_v], "MaxSteeringAngle", obj.max_gamma, "VehicleInputs",  "VehicleSpeedSteeringAngle");
            %%% END MODEL HERE %%%
        end
        
        function obj = drive(obj, v, gamma, dt)
            % https://www.mathworks.com/help/nav/ref/bicyclekinematics.derivative.html
            gamma = min(max(gamma, -obj.max_gamma), obj.max_gamma);
            dot = derivative(obj.systemModel, [obj.x, obj.y, obj.theta], [v, gamma]);
            
            obj.x = obj.x + dot(1) * dt;
            obj.y = obj.y + dot(2) * dt;
            obj.theta = mod(obj.theta + dot(3) * dt, 2*pi);
            if obj.theta > pi
                obj.theta = obj.theta - 2*pi;
            end
            obj.v = v;
            obj.gamma = gamma;

        end
        
        function [x, y, theta, obj] = odom(obj)
            x = obj.x;
            y = obj.y;
            theta = obj.theta;
        end
    end
    
end