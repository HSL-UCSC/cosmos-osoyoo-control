%% system model v3 using bicycle model
% using https://www.mathworks.com/help/nav/ref/bicyclekinematics.html

classdef Model
    properties
        wheel_base = 11;
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
            obj.systemModel = bicycleKinematics("WheelBase", obj.wheel_base, "VehicleSpeedRange", [0, obj.max_v], "MaxSteeringAngle", obj.max_gamma, "VehicleInputs",  "VehicleSpeedSteeringAngle");
        end
        
        function obj = drive(obj, v, gamma, dt)
            % https://www.mathworks.com/help/nav/ref/bicyclekinematics.derivative.html
            dot = derivative(obj.systemModel, [obj.x, obj.y, obj.theta], [v, gamma]);
            obj.x = obj.x + dot(1) * dt;
            obj.y = obj.y + dot(2) * dt;
            obj.theta = mod(obj.theta + dot(3) * dt, 2*pi);
            obj.v = v;
            obj.gamma = gamma;

            [v gamma]
            dot

            % % send command to car
            % Lspeed = v - (gamma * obj.wheel_base / 2);
            % Rspeed = v + (gamma * obj.wheel_base / 2);
            % 
            % % Scale speeds to uint8 range (0-255)
            % Lspeed = (max(min(Lspeed * 255 / obj.max_v, 255), 0));
            % Rspeed = (max(min(Rspeed * 255 / obj.max_v, 255), 0));
            % [Lspeed, Rspeed]
        end
        
        function [x, y, theta, obj] = odom(obj)
            x = obj.x;
            y = obj.y;
            theta = obj.theta;
        end
    end
    
end