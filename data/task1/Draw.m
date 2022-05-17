close all;
data1 = matfile("theta1pi2theta2pi3theta3pi3.mat");
angles1 = data1.theta1pi2theta2pi3theta3pi3;
data2 = matfile("theta1_pi2theta2pi3theta3pi3.mat");
angles2 = data2.theta1_pi2theta2pi3theta3pi3;
data3 = matfile("theta1pi3theta2pi4theta3pi4.mat");
angles3 = data3.theta1pi3theta2pi4theta3pi4;

for ind = 1 : length(angles1)
   %disp(angles1(ind,:));
   time = angles1(:,1);
   theta1 = angles1(:,2);
   theta2 = angles1(:,3);
   theta3 = angles1(:,4);
end
figure(1);
hold on ;
plot(time, theta1, 'Color', 'r', 'LineWidth', 2);
plot(time, theta2, 'Color', 'g', 'LineWidth', 2);
plot(time, theta3, 'Color', 'b', 'LineWidth', 2);
legend 'Theta1' 'Theta2' 'Theta3';
title('Target $\theta_1 = -\frac{\pi}{2}$, $\theta_2 = \frac{\pi}{3}$, $\theta_3 = \frac{\pi}{3}$ ','Interpreter','latex') 
xlabel('Time ($s$)','Interpreter','latex');
ylabel('Angle ($rad$)','Interpreter','latex');
hold off;
line = [];
for ind = 1: length(angles1)
    [x, y, z] = FowardKinematics(theta1(ind), theta2(ind), theta3(ind));
    line = [line; x, y, z];
end
hold off;
figure(4);
plot3(line(:,1), line(:,2), line(:,3), 'r', 'LineWidth', 2);
xlabel('X ($m$)','Interpreter','latex');
ylabel('Y ($m$)','Interpreter','latex');
zlabel('Z ($m$)','Interpreter','latex');
legend 'Target 1';
text(line(end,1), line(end,2), line(end,3), 'Target 1');
hold on;



for ind = 1 : length(angles2)
   %disp(angles1(ind,:));
   time = angles2(:,1);
   theta1 = angles2(:,2);
   theta2 = angles2(:,3);
   theta3 = angles2(:,4);
end
figure(2);
hold on ;
plot(time, theta1, 'Color', 'r', 'LineWidth', 2);
plot(time, theta2, 'Color', 'g', 'LineWidth', 2);
plot(time, theta3, 'Color', 'b', 'LineWidth', 2);
legend 'Theta1' 'Theta2' 'Theta3';
title('Target $\theta_1 = \frac{\pi}{2}$, $\theta_2 = \frac{\pi}{3}$, $\theta_3 = \frac{\pi}{3}$ ','Interpreter','latex') 
xlabel('Time ($s$)','Interpreter','latex');
ylabel('Angle ($rad$)','Interpreter','latex');
hold off;
line = [];
for ind = 1: length(angles2)
    [x, y, z] = FowardKinematics(theta1(ind), theta2(ind), theta3(ind));
    line = [line; x, y, z];
end
hold off;
figure(4);
plot3(line(:,1), line(:,2), line(:,3), 'g', 'LineWidth', 2);
xlabel('X ($m$)','Interpreter','latex');
ylabel('Y ($m$)','Interpreter','latex');
zlabel('Z ($m$)','Interpreter','latex');
legend 'Target 2';
text(line(end,1), line(end,2), line(end,3), 'Target 2');
hold on;




for ind = 1 : length(angles3)
   %disp(angles1(ind,:));
   time = angles3(:,1);
   theta1 = angles3(:,2);
   theta2 = angles3(:,3);
   theta3 = angles3(:,4);
end
figure(3);
hold on ;
plot(time, theta1, 'Color', 'r', 'LineWidth', 2);
plot(time, theta2, 'Color', 'g', 'LineWidth', 2);
plot(time, theta3, 'Color', 'b', 'LineWidth', 2);
legend 'Theta1' 'Theta2' 'Theta3';
title('Target $\theta_1 = -\frac{\pi}{3}$, $\theta_2 = \frac{\pi}{4}$, $\theta_3 = \frac{\pi}{4}$ ','Interpreter','latex') 
xlabel('Time ($s$)','Interpreter','latex');
ylabel('Angle ($rad$)','Interpreter','latex');
hold off;

line = [];
for ind = 1: length(angles3)
    [x, y, z] = FowardKinematics(theta1(ind), theta2(ind), theta3(ind));
    line = [line; x, y, z];
end

hold off;
figure(4);
plot3(line(:,1), line(:,2), line(:,3), 'b', 'LineWidth', 2);
xlabel('X ($m$)','Interpreter','latex');
ylabel('Y ($m$)','Interpreter','latex');
zlabel('Z ($m$)','Interpreter','latex');
legend 'Target 1' 'Target 2' 'Target 3';
text(line(end,1), line(end,2), line(end,3), 'Target 3');
grif on;
hold on;

