function [teta1,teta2,d] = ik(x,y,z,l1,l2)
% Written by Raþit EVDÜZEN
% 11-Jun-2019 13:56:15
% 2 dof robotic arm inverse and forward kinematic solution with geometric
% approach. Ýn this code 2 different approach for inverse kinematic
% solution.if you one of solution needed go the equation temp2 and temp1
% and change the sign (ex. - => + ), the other one needed the solution
% opposite the change solution one
% x => target x position
% y => target y position
% z0 => current z position
% z1 => target z position
% l1 => Robot link length
% l2 => Robot link length



if (sqrt(x^2 + y^2)) > (l1+l2) 
    warning("Wrong Set Point");
else
    a = (x^2 + y^2 - l1^2 -l2^2)/(2*l1*l2);
    temp2 = atan2((sqrt(1-(a)^2)),(a));   % Another Solve - = +
    teta2 = rad2deg(temp2);
    temp1 = atan2(y,x) - atan2((sqrt(y^2 + x^2 - (l2*cos(temp2)+l1)^2)),(l2*cos(temp2)+l1));   % Another Solve + = -
    teta1 = rad2deg(temp1);
    d = z;
    % test for each joint angle than solve the forward kinematics
    % xhat = l1 * cos(temp1) + l2 * cos(temp1 + temp2);
    % yhat = l1 * sin(temp1) + l2 * sin(temp1 + temp2);
end

