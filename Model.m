%% system model v3 using bicycle model
% using https://www.mathworks.com/help/nav/ref/bicyclekinematics.html

classdef Model
    properties
        % bicycle model
        systemModel = bicycleKinematics("WheelBase", 0.162, "VehicleSpeedRange", [-10, 10], "MaxSteeringAngle", pi / 3, "VehicleInputs", "VehicleSpeedSteeringAngle");
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
        end
        
        function obj = drive(obj, v, gamma, dt)
            % https://www.mathworks.com/help/nav/ref/bicyclekinematics.derivative.html
            dot = derivative(obj.systemModel, [obj.x, obj.y, obj.theta], [v, gamma]);
            obj.x = obj.x + dot(1) * dt;
            obj.y = obj.y + dot(2) * dt;
            obj.theta = mod(obj.theta + dot(3) * dt, 2*pi);
            obj.v = v;
            obj.gamma = gamma;
        end        
        function obj = driveOn(obj, dt)
            obj = obj.drive(0, obj.gamma, dt);
        end
        function [x, y, theta, obj] = odom(obj)
            x = obj.x;
            y = obj.y;
            theta = obj.theta;
        end
    end
    
end