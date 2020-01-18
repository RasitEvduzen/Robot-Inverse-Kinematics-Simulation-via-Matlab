%% Load Data
load data
% Robot UR-(5) DH parameters
alpha = [0, 90, 0, 0, 90, -90];
a     = [0,0,-0.425, -0.39225, 0,0];
d     = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];
theta = [theta1, theta2, theta3,theta4,theta5,theta6];
a = a * 1e3;    % scaling for link parameters
d = d * 1e3;
a2 = a(1,3);a3 = a(1,4);d1 = d(1,1);d4 = d(1,4);d5 = d(1,5);d6 = d(1,6);
% Calculate Transformation Matrix for each joint
for i=1:length(theta)
    T(:,:,i) = DHMatrix(alpha(i),a(i),d(i),theta(i));
end



%% Calculate ÝK - UR5 ROBOTÝC ARM

% Theta-1  [2x solution]
P05 = T_end*[0 0 -d6 1]';
f1 = atan2(P05(2,1),P05(1,1)); 
f2 = acos(d4/(sqrt(P05(1,1)^2+P05(2,1)^2)));  % + -
itheta1 = (f1*180/pi) + (f2*180/pi) + 90;         % There are two solution here left - right

% Theta - 5 [2x solution]
P06x = T_end(1,4); P06y = T_end(2,4);
itheta5 = acos([P06x*sin(itheta1*pi/180) - P06y*cos(itheta1*pi/180) - d4] / d6); % + -
itheta5 = (itheta5*180/pi);     % There are two solution wrist up - down

% Theta - 6  [1x solution]
T60 = [inv(T_end(1:3,1:3)) -inv(T_end(1:3,1:3))*T_end(1:3,4); 0 0 0 1];   % inv(T_end)
A1 = [-T60(2,1)*sin(itheta1*pi/180) + T60(2,2)*cos(itheta1*pi/180)] / [sin(itheta5*pi/180)];
A2 = [T60(1,1)*sin(itheta1*pi/180) - T60(1,2)*cos(itheta1*pi/180)] / [sin(itheta5*pi/180)];
itheta6 = atan2(A1,A2);
itheta6 = (itheta6*180/pi);

% Theta - 3  [2x solution]
T01 = DHMatrix(alpha(1),a(1),d(1),theta(1));
T16 = inv(T01) * T_end;
T45 = DHMatrix(alpha(5),a(5),d(5),theta(5));
T56 = DHMatrix(alpha(6),a(6),d(6),theta(6));
T14 = T16 * inv(T45*T56);
P14xz = sqrt(T14(1,4)^2 + T14(3,4)^2);
A = [(P14xz)^2 - a2^2 - a3^2] / [(2*a2*a3)];
itheta3 = acos(A); %+ -
itheta3 = (itheta3*180/pi);     % There are two solution wrist up - down

% Theta - 2  [1x solution] carefully 
% T12 = DHMatrix(alpha(2),a(2),d(2),theta(2));
% T23 = DHMatrix(alpha(3),a(3),d(3),theta(3));
% T34 = DHMatrix(alpha(4),a(4),d(4),theta(4));
% T14 = T12*T23*T34;
P14z = T14(3,4);
P14x = T14(1,4);
fi1 = atan2(-P14z,-P14x);
fi1 = (fi1*180/pi);
tempfi2 = [(-a3*sin(itheta3*pi/180))]/[(P14xz)];
fi2 = asin(tempfi2);
fi2 = (fi2*180/pi);
itheta2 = fi1 -  fi2;

% Theta - 4  [1x solution]
T34 = DHMatrix(alpha(4),a(4),d(4),theta(4));
itheta4 = atan2(-T34(1,2),T34(1,1));   % Correct Solution - , + solution is wrong
itheta4 = (itheta4*180/pi);


%% PLOT
itheta = [itheta1 itheta2 itheta3 itheta4 itheta5 itheta6];
% Robot UR-(5) DH parameters
alpha = [0, 90, 0, 0, 90, -90];
a     = [0,0,-0.425, -0.39225, 0,0];
d     = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];
theta = itheta;
a = a * 1e3;    % scaling for link parameters
d = d * 1e3;
a2 = a(1,3);a3 = a(1,4);d1 = d(1,1);d4 = d(1,4);d5 = d(1,5);d6 = d(1,6);
% Calculate Transformation Matrix for each joint
for i=1:length(theta)
    T(:,:,i) = DHMatrix(alpha(i),a(i),d(i),theta(i));
end
% Calculate Forward Kinematics and Simulation
T_end = eye(4,4);
for k=1:length(theta)
    temp = T_end;
    T_end = T_end* T(:,:,k);
    % Simulation
    plot3([temp(1,4) T_end(1,4)],[temp(2,4) T_end(2,4)],[temp(3,4) T_end(3,4)],'r--','LineWidth',3);
    hold on,grid on ,axis([-1000 1000 -1000 1000 -1000 1000])
    scatter3(T_end(1,4),T_end(2,4),T_end(3,4),'MarkerFaceColor',[1 0 0]);
    scatter3(0,0,0,'MarkerFaceColor',[1 0 0]);
    xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('UR-5 Robot manipulator simulation ')
end
str1 = {['X : ',num2str(T_end(1,4)),' Y : ',num2str(T_end(2,4)),' Z : ',num2str(T_end(3,4))]};
str2 = {['X : ',num2str(0),' Y : ',num2str(0),' Z : ',num2str(0)]};
text(T_end(1,4)+10,T_end(2,4),T_end(3,4),str1,'Color','k','FontSize',10)
text(0,0,0,str2,'Color','k','FontSize',10)