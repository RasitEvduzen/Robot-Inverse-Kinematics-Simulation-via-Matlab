function [t1,t2,t3] = ik(px,py,pz,l1,l2,l3)
% This function solve 3 dof RRR type robotic arm inverse kinematics via
% analytic approach
% Ýnput parameters = robot position cartesian space x,y,z and robot physical
% configration l1,l2,l3 = link length
% Output parameters = 3 dof robotic arm inverse kinematic solution each
% joint calculate analyticly approach

% Calculate theta3
d1 = l1;
c3 = [(px^2) + (py^2) + (pz - d1)^2 - (l2^2) - (l3^2)] / [2*l2*l3];
s3 = sqrt(1 - (c3^2));
theta3 = atan2(-s3,c3);    % another solution (+s3,c3)


% Calculate theta1 
theta1 = atan2(py,px);   % another solution (-py,-px)

% Calculate theta2 
% This joint angle2 calculation, we use the linear system approach Ax=b
A = [(l2+l3*cos(theta3)) (-l3*sin(theta3)); (l3*sin(theta3)) (l2+l3*cos(theta3))];
b = [(cos(theta1)*px + sin(theta1)*py); (pz-d1)];
S = inv(A)*b;  
c2 = S(1,:);
s2 = S(2,:);
theta2 = atan2(s2,c2);      % four regular solution theta2 depending on combinations of (+,-) s2,s3

t3 = (theta3) * (180/pi);  % convert radian to degree
t1 = (theta1) * (180/pi);
t2 = (theta2) * (180/pi);
end


