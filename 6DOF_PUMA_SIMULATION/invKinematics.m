function [theta1,theta2,theta3,theta4,theta5,theta6] = invKinematics(T_end)
% Written By: Raþit EVDÜZEN
% 23-Dec-2019 16:46:08

% DH - Parameters
a2 = 650;
a3 = 0;
d3 = 190;
d4 = 600;
d6 = 125;

% End Effector Position & Orientation
Px = T_end(1,4);
Py = T_end(2,4);
Pz = T_end(3,4);
r11 = T_end(1,1);
r21 = T_end(2,1);
r31 = T_end(3,1);
r13 = T_end(1,3);
r23 = T_end(2,3);
r33 = T_end(3,3);

% Ýnverse Theta - 1
K = (Px^2+Py^2+Pz^2-a2^2-a3^2-d3^2-d4^2)/(2*a2);
theta1 = (atan2(Py,Px)-atan2(d3,sqrt(Px^2+Py^2-d3^2)));
c1 = cos(theta1);
s1 = sin(theta1);

% Ýnverse Theta - 3
theta3 = (atan2(a3,d4)-atan2(real(K),real(sqrt(a3^2+d4^2-K^2))));
c3 = cos(theta3);
s3 = sin(theta3);

% Ýnverse Theta - 2
t23 = atan2((-a3-a2*c3)*Pz-(c1*Px+s1*Py)*(d4-a2*s3),(a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py));
theta2 = (t23 - theta3);
c2 = cos(theta2);
s2 = sin(theta2);
s23 = ((-a3-a2*c3)*Pz+(c1*Px+s1*Py)*(a2*s3-d4))/(Pz^2+(c1*Px+s1*Py)^2);
c23 = ((a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py))/(Pz^2+(c1*Px+s1*Py)^2);

% Ýnverse Theta - 4
theta4 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23 + r33*s23);
c4 = cos(theta4);
s4 = sin (theta4);

% Ýnverse Theta - 5
s5 = -(r13*(c1*c23*c4+s1*s4)+r23*(s1*c23*c4-c1*s4)-r33*(s23*c4));
c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
theta5 = atan2(s5,c5);

% Ýnverse Theta - 6
s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*(s23*s4);
c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
theta6 = atan2(s6,c6);
% Rad to Deg
theta1 = theta1*180/pi;
theta2 = theta2*180/pi;
theta3 = theta3*180/pi;
theta4 = theta4*180/pi;
theta5 = theta5*180/pi;
theta6 = theta6*180/pi;

end