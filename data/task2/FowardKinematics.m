function [X, Y, Z] = FowardKinematics(Theta1, Theta2, Theta3)

a1 = 6;
a2 = 15;
a3 = 14.5;

d1 = 16.3;
d2 = 0;
d3 = 0;

T10 = [cos(Theta1),      0,   sin(Theta1), a1*cos(Theta1);
       sin(Theta1),      0,  -cos(Theta1), a1*sin(Theta1);
           0,            1,       0,            d1;
           0,            0,       0,            1      ];

T21 = [cos(Theta2+ pi/2), -sin(Theta2 + pi/2),   0,     a2*cos(Theta2+ pi/2);
       sin(Theta2+ pi/2), cos(Theta2 + pi/2),    0,     a2*sin(Theta2+ pi/2);
             0,                  0,              1,              0          ;
             0,                  0,              0,              1         ];
T32 = [cos(Theta3), -sin(Theta3),   0,     a3*cos(Theta3);
       sin(Theta3), cos(Theta3),    0,     a3*sin(Theta3);
           0,            0,         1,         0         ;
           0,            0,         0,         1         ];
             
T30 = T10*T21*T32;

X = T30(1,4);
Y = T30(2,4);
Z = T30(3,4);