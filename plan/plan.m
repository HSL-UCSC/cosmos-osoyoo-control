clear;
close all;
% populate plan.mat

% create data points to form a circle to store in plan.mat
% circle is centered at (0,0) with radius 1.5
% circle is divided into 100 points

% create a vector of 100 points from 0 to 2*pi
% theta = linspace(0,2*pi,60+1);
% plot(theta)

% data = [cos(theta-pi/2); -1*sin(theta-pi/2)].*1.5;

x = linspace(0, 2*pi, 60+1);
data = [(x.*(3 / (2*pi)))-1; sin(x).*((1.75+1)/2)-((1.75-1)/2)];

plot(data(1,:),data(2,:), '-o')

save('plan.mat', 'data');