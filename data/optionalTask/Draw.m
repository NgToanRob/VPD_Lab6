figure(1);
X0 = [-5; 5; 0];
X1 = [-10; -10; 30];
t = linspace(0,5,100);
X = X0 + (X1 - X0) * t ./ 5;
plot3(X(1,:), X(2,:), X(3,:), 'LineWidth',2);
xlim([-15 0]);
ylim([-15 10]);
zlim([0 35]);
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
title('Trajectory');
grid on;

simm = sim("Arm3dof_ver2017a.slx");
anglesFromSensor = simm.anglesFromSensor;
positionFromSensor = simm.realPosition;
estimatorPosition = simm.estimatorPosition;
% Angles From sensors
figure(2);
hold on;
plot(anglesFromSensor.Time, anglesFromSensor.data(:,1), 'r', 'LineWidth',2);
plot(anglesFromSensor.Time, anglesFromSensor.data(:,2), 'g', 'LineWidth',2);
plot(anglesFromSensor.Time, anglesFromSensor.data(:,3), 'b', 'LineWidth',2);
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Angles from sensors');
legend 'Theta 1' 'Theta 2' 'Theta 3';
grid on;


% Positions From Sensors
figure(3);
plot3(positionFromSensor.data(:,1), positionFromSensor.data(:,2), positionFromSensor.data(:,3), 'LineWidth',2);
xlim([-15 0]);
ylim([-15 10]);
zlim([0 35]);
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
title('Positions From Sensors');
grid on;

% Estimator
figure(4);
plot3(estimatorPosition.data(:,1), estimatorPosition.data(:,2), estimatorPosition.data(:,3), 'LineWidth',2);
xlim([-15 0]);
ylim([-15 10]);
zlim([0 35]);
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
title('Estimator Position');
grid on;