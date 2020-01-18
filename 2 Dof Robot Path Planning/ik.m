function [teta1,teta2] = ik(x,y,l1,l2)
% Written by Raþit EVDÜZEN
% 11-Jun-2016
% 2 dof robotic arm inverse and forward kinematic solution with geometric
% approach. Ýn this code 2 different approach for inverse kinematic
% solution.if you one of solution needed go the equation temp2 and temp1
% and change the sign (ex. - => + ), the other one needed the solution
% opposite the change solution one


if (sqrt(x^2 + y^2)) > (l1+l2)
    warning("Wrong Set Point")
else
    a = (x^2 + y^2 - l1^2 -l2^2)/(2*l1*l2);
    b = sqrt(1-(a)^2);
    temp2 = atan2((-b),(a));   % Solution 1 (-b), Solution 2 (b)
    teta2 = rad2deg(temp2);
    temp1 = atan2(y,x) + atan2((sqrt(y^2 + x^2 - (l2*cos(temp2)+l1)^2)),(l2*cos(temp2)+l1));   
    % Solution 1 (+ atan2), Solution 2 (- atan2)
    teta1 = rad2deg(temp1);
    
    % test for each joint angle than solve the forward kinematics
    % xhat = l1 * cos(temp1) + l2 * cos(temp1 + temp2);
    % yhat = l1 * sin(temp1) + l2 * sin(temp1 + temp2);
end

