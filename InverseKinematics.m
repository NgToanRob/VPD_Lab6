function [Theta1, Theta2, Theta3] = InverseKinematics(Xd, Yd, Zd)

a1 = 6;
a2 = 15;
a3 = 14.5;

d1 = 16.3;
d2 = 0;
d3 = 0;

Theta1 = atan2(Yd,Xd);

r1 = sqrt((Xd - a1*cos(Theta1))^2 + (Yd - a1*sin(Theta1))^2 );
r2 = Zd -d1;
Phi2 = atan2(r2,r1);

r3 = sqrt(r1^2 + r2^2);
Phi1 = acos((a2^2+r3^2-a3^2)/(2*a2*r3));

Theta2 = pi/2 - (Phi1 + Phi2);


Phi3 = acos((a2^2+a3^2-r3^2)/(2*a2*a3));

Theta3 = pi - Phi3;
