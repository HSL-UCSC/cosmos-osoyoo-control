clear;
close all;

DIST_THRESHOLD = 0.5;
ARROW_SCALE = 0.2;
DT = 0.4;

% reference plan (imported from plan.m --> plan.mat)
r_plan = matfile('plan_20_circle.mat').data;

% car = Model();
 car = Car();
 %car = Dog(false);
 car.x = -1;
 car.y = -1;
% controller = Basic_Control();
% controller = PID_Control();
% controller = MPC_Control(DT);
% controller = PurePursuit_Control(r_plan', 1); % lookahead distance 0.5
% for car, 1 for dog
controller = CustomPurePursuit_Control(r_plan', 1, DIST_THRESHOLD);

plot(r_plan(1,:),r_plan(2,:));

 disp(r_plan(1, 1));
 disp(r_plan(2, 1));

% index of the next point on the reference plan (starts as the first point)
index = 1;

recorded_data = [];

while true
    
    [x, y, theta, car] = car.odom();
    
    % find if the car is close enough to the next point on the reference plan
    [x_target, y_target, theta_target, index] = motion_plan(x, y, theta, r_plan, index, DIST_THRESHOLD);
    [done, controller] = controller.done();
    if index == -1 || done
        break;
    end

    plot(r_plan(1,:), r_plan(2,:), '-o', 'Color', 'k');
    xlabel('X')
    ylabel('Y')
    xlim([-3 3])
    axis equal;
    hold on;
    
    % move the car to the next point on the reference plan
%     car = car.driveOn(DT);
    controller = controller.update(x, y, theta, x_target, y_target, theta_target);
    [v, gamma, controller] = controller.get_control();
    car = car.drive(v, gamma, DT);
    
    % record the data
    recorded_data = [recorded_data; x, y, theta, x_target, y_target, theta_target, index, v, gamma];
    
    % plot the car
    plot(x_target, y_target, '.', 'Color', 'r', 'MarkerSize', 20);
    quiver(x, y, ARROW_SCALE*cos(theta), ARROW_SCALE*sin(theta), 'Color', 'magenta', 'MaxHeadSize', ARROW_SCALE);
    quiver(x, y, ARROW_SCALE*cos(theta_target), ARROW_SCALE*sin(theta_target), 'Color', 'cyan', 'MaxHeadSize', ARROW_SCALE);
    plot(recorded_data(:,1), recorded_data(:,2), 'Color', 'b');
    if isa(controller, 'MPC_Control') || isa(controller, 'PurePursuit_Control')
        plot(controller.prediction(:,1), controller.prediction(:,2), 'Color', 'g');
        plot(controller.prediction(end,1), controller.prediction(end,2), 'Color', 'g', 'Marker','x', 'MarkerSize', 20);
    end
    hold off;
    drawnow;

    %pause(DT/2);

end
car.drive(0, 0, 0);
% heading & target heading over time
% distance from target position over time
% x over time
% y over time
% speed over time

figure;
subplot(2,1,1);
plot(recorded_data(:,1));
title('Car X');
xlabel('Time');
ylabel('X');

subplot(2,1,2);
plot(recorded_data(:,2));
title('Car Z');
xlabel('Time');
ylabel('Z');

figure;
subplot(2,1,1);
plot(recorded_data(:,8));
title('Car Velocity');
xlabel('Time');
ylabel('Velocity');

subplot(2,1,2);
plot(recorded_data(:,9));
title('Car Steering Angle');
xlabel('Time');
ylabel('Steering Angle (rad)');

figure;
plot(recorded_data(:,3));
hold on;
plot(recorded_data(:, 6));
title("Heading vs. Target Heading");
legend('Heading', 'Target Heading');
xlabel('Time');
ylabel('Heading (rad)');
hold off;

figure;
dist = abs(hypot(recorded_data(:,5)-recorded_data(:,2), recorded_data(:,4)-recorded_data(:,1)));
plot(dist);
title('Distance from Target Position');
xlabel('Time');
ylabel('Distance');