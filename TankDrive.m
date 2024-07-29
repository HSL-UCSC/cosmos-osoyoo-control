classdef TankDrive
    properties
        x
        y
        theta
        udp
    end
    
    methods
        function obj = TankDrive()
            obj.x = 0;
            obj.y = 0;
            obj.theta = 0;
            obj.udp = udpport;
        end
        
        function obj = drive(obj, command, duration)
            % Static IP and port for the car
            ip = '192.168.0.161';
            port = 8888;
            
            % Send the command to the car via UDP
            write(obj.udp, command, 'uint8', ip, port);
            pause(duration);
            % Stop the car
            write(obj.udp, "E", 'uint8', ip, port);
        end
        
        function obj = updatePosition(obj, x, y, theta)
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
        end
    end
end
