classdef CustomPurePursuit < handle
    properties
        LookaheadDistanceBase = 1.0; % Base lookahead distance, remains constant
        LookaheadSpeedGain = 0.1; % Gain for speed-dependent lookahead adjustment
        MaxAngularVelocity = 1.0; % Maximum angular velocity
        DesiredLinearVelocity = 1.0; % Desired linear velocity
        Waypoints = []; % Original waypoints provided by the user
        InterpolatedWaypoints = []; % Waypoints after spline interpolation for a smoother path
        Speed = 0; % Current speed of the vehicle
        DynamicLookaheadDistance; % Calculated dynamic lookahead distance based on speed
        WaypointIndex = 1;
    end
    
    methods
        function obj = CustomPurePursuit(varargin)
            % Constructor that accepts name-value pairs for setting properties
            for i = 1:2:length(varargin)
                if i + 1 <= length(varargin)
                    propName = varargin{i};
                    propValue = varargin{i + 1};
                    switch lower(propName)
                        case 'lookaheaddistance'
                            obj.LookaheadDistanceBase = propValue;
                        case 'maxangularvelocity'
                            obj.MaxAngularVelocity = propValue;
                        case 'desiredlinearvelocity'
                            obj.DesiredLinearVelocity = propValue;
                        case 'waypoints'
                            obj.Waypoints = propValue;
%                             disp("waypoints")
                            obj = obj.interpolateWaypoints(); % Interpolate waypoints after setting
                        otherwise
                            warning('Unknown property name: %s', propName);
                    end
                end
            end
            % Initialize DynamicLookaheadDistance with the base value
            obj.DynamicLookaheadDistance = obj.LookaheadDistanceBase;
        end
        
        function obj = interpolateWaypoints(obj)
            % Spline interpolation for smoother path generation
            if ~isempty(obj.Waypoints) && size(obj.Waypoints, 1) > 2
                x = obj.Waypoints(:, 1);
                y = obj.Waypoints(:, 2);
                t = 1:length(x);
                tt = linspace(1, length(x), 10 * length(x)); % Increase points
                xi = spline(t, x, tt);
                yi = spline(t, y, tt);
                obj.InterpolatedWaypoints = [xi', yi'];
            else
                obj.InterpolatedWaypoints = obj.Waypoints; % Not enough points for spline
            end
        end
        
        function [v, omega, obj] = control(obj, currentPose, currentSpeed)
            % Calculate control commands based on the current pose and speed
            obj.Speed = currentSpeed;
            obj = obj.adjustLookaheadDistance(); % Adjusts and updates DynamicLookaheadDistance based on current speed
            [lookAheadPoint, found, obj] = obj.findLookAheadPoint(currentPose);
            plot(lookAheadPoint(1), lookAheadPoint(2), '+', 'Color', 'g', 'MarkerSize', 20);
%             plot(currentPose(1) + obj.DynamicLookaheadDistance * cos(linspace(0, 2*pi)), currentPose(2) + obj.DynamicLookaheadDistance * sin(linspace(0, 2*pi)));
%             disp("pt")
%             disp(lookAheadPoint)
            if ~found
%                 disp("help!")
                v = 0;
                omega = 0;
                return;
            end
            
            angleToGoal = atan2(lookAheadPoint(2) - currentPose(2), lookAheadPoint(1) - currentPose(1));
            steeringAngle = angleToGoal - currentPose(3);
            steeringAngle = atan2(sin(steeringAngle), cos(steeringAngle));

%             disp(angleToGoal)
%             disp(steeringAngle)
            
%             omega = (2 * obj.DesiredLinearVelocity / obj.DynamicLookaheadDistance) * sin(steeringAngle);
%             omega = max(min(omega, obj.MaxAngularVelocity), -obj.MaxAngularVelocity);
            v = obj.DesiredLinearVelocity;
            omega = steeringAngle;
        end
        
        function obj = adjustLookaheadDistance(obj)
            % Dynamically adjusts the lookahead distance based on the current speed
            obj.DynamicLookaheadDistance = obj.LookaheadDistanceBase + obj.LookaheadSpeedGain * sqrt(obj.Speed);
        end
        
        function [lookAheadPoint, found, obj] = findLookAheadPoint(obj, currentPose)
            % Find the lookahead point on the interpolated path using DynamicLookaheadDistance
%             disp("func?")
            found = false;
            lookAheadPoint = [0, 0];
            minDist = inf;

%             disp(size(obj.InterpolatedWaypoints, 1))
%             disp("pose")
%             disp(currentPose(1:2))
            
            for i = obj.WaypointIndex:min(size(obj.InterpolatedWaypoints, 1), obj.WaypointIndex+20)
                point = obj.InterpolatedWaypoints(i, :);
%                 segmentEnd = obj.InterpolatedWaypoints(i + 1, :);
                
%                 [point, isOnSegment] = obj.projectLookAheadPoint(currentPose, segmentStart, segmentEnd);
%                 plot(point(1), point(2), 'x', 'Color', 'r', 'MarkerSize', 10);

%                 if isOnSegment
%                     plot(point(1), point(2), 'x', 'Color', 'r', 'MarkerSize', 10);
                    dist = abs(norm(currentPose(1:2) - point) - obj.DynamicLookaheadDistance);
                    
                    if dist < minDist % && dist <= obj.DynamicLookaheadDistance
                        minDist = dist;
                        lookAheadPoint = point;
                        found = true;
                        obj.WaypointIndex = i;
                    end
%                 end
            end
%             disp("dist")
%             disp(minDist)
        end
        
        function [point, isOnSegment] = projectLookAheadPoint(~, currentPose, segmentStart, segmentEnd)
            % Vector projection to find the closest point on the segment to the current position
            a = currentPose(1:2) - segmentStart;
            b = segmentEnd - segmentStart;
            t = dot(a, b) / norm(b);
            isOnSegment = t >= 0 && t <= norm(b);
%             t = max(0, min(1, t)); % Clamp t to the segment
            point = segmentStart + t * b;
        end
    end
end
