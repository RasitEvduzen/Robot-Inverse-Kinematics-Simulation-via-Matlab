clc,clear all,close all;
%% UNÝVERSAL ROBOT - 5 FORWARD and ÝNVERSE KÝNEMATÝCS ANALYSÝS
% Written By: Raþit EVDÜZEN 
% 01-Dec-2019 
set(gcf,'Position',[100 100 1720 900])

N = 20; % Simulation Time
for i=1:N
    clf
%% Calculate FK - UR5 ROBOTÝC ARM 
% Joint Parameters

theta1 = 90*rand();
theta2 = 90*rand();
theta3 = 90*rand();
theta4 = 90*rand();
theta5 = 90*rand();
theta6 = 90*rand();

% Robot UR-(5) DH parameters
alpha = [0, 90, 0, 0, 90, -90];
a     = [0,0,-0.425, -0.39225, 0,0];
d     = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];
theta = [theta1, theta2, theta3,theta4,theta5,theta6]; ftheta = theta;
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
    plot3([temp(1,4) T_end(1,4)],[temp(2,4) T_end(2,4)],[temp(3,4) T_end(3,4)],'b','LineWidth',3);
    hold on,grid on ,axis([-1000 1000 -1000 1000 -1000 1000])
    scatter3(T_end(1,4),T_end(2,4),T_end(3,4),'MarkerFaceColor',[1 0 0]);
    scatter3(0,0,0,'MarkerFaceColor',[1 0 0]);
    xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('UR-5 Robot manipulator simulation')
end
str1 = {['X : ',num2str(T_end(1,4)),' Y : ',num2str(T_end(2,4)),' Z : ',num2str(T_end(3,4))]};
str2 = {['X : ',num2str(0),' Y : ',num2str(0),' Z : ',num2str(0)]};
text(T_end(1,4)+10,T_end(2,4),T_end(3,4),str1,'Color','k','FontSize',10)
text(0,0,0,str2,'Color','k','FontSize',10)
save data T_end
run ik
pause(0.1)
end
ftheta
itheta




