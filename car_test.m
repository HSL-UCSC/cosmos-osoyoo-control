clear;
close all;

car = Car();

% i = 1;
% 
% headings = linspace(-pi/4, pi/4, 200);
% 
% while(i < 200)
%     [x, y, theta, car] = car.odom()
% 
%     disp(headings(i))
% 
%     car.drive(120, headings(i));
%     i = i+1;
%     pause(0.05);
% end

car.drive(0, 0);

% speeds = linspace(0, 0.1, 300);
% 
% i = 1;
% while (i < 300)
%     disp(speeds(i));
%     car.drive(speeds(i), 0, 1);
%     i = i+1;
%     pause(0.1);
% end
% car.drive(0.1,pi/4, 1);