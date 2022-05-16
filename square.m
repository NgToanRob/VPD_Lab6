close all;
hold on

% Motor constants
J = 0.0023;
L = 0.0047;
R_electronic = 4.73;
k_e = 0.274;
k_m = k_e;
M_oth = 0;

% Controler
R = 0.025;
B = 0.162;
K_str = 300;
K_rot = 500;
U_max = 3.35;

%axises
p = line([0 0],[-2 2]);
p.Color = 'red';
p.LineStyle = '--';
p = line([-2 2],[0 0]);
p.Color = 'red';
p.LineStyle = '--';

%starting position of robot
start_x = 0;
start_y = 0;
start_angel = 0;

%enter goals
x = [1, 0, 0, 0];
y = [0, 0, 1, 0];

for i = 1: length(x)
    goal_x = x(i);
    goal_y = y(i);
    simm = sim("model2.slx");

    plot(simm.X.data, simm.Y.data ,'green' ,'Linewidth',2);

    start_angel = simm.angel.Data(end);
    start_x = goal_x;
    start_y = goal_y;
    %start_angel = 0;
    %start_x = 0;
    %start_y = 0;
end

% data = readmatrix('fixdata/data_square.txt');
% x = data(:,2);
% y = data(:,1);
% plot(x,y, 'g',  'Linewidth',3 );
%legend('','', 'Model');
title('Using non-linear controller Ks = 300, Kr = 500')
xlabel('x (m)');
ylabel('y (m)');