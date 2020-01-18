clc,clear all,close all;
% Written By: Rasit EVDÜZEN
% '2-Jan-2020'
set(gcf,'Position',[100 100 1720 900])

% First this code get the each joint angle,than calculate robot forward
% model based DH parameter approach.Next save the end effector
% transformation matrix for inverse kinematic calculation

   
%% Robot Configration 6 Dof Small Robot  
% Joint Parameters
%---------------Random Ýnput% 
theta1 = 90*rand-45;
theta2 = 90*rand-45;   
theta3 = 90*rand-45;
theta4 = 90*rand-45;
theta5 = 90*rand-45;
theta6 = 90*rand-45;
%--------------------------%
% theta1 = 0;
% theta2 = 0;   
% theta3 = 0;
% theta4 = 0;
% theta5 = 0;
% theta6 = 0;

% Robot DH parameters 
alpha = [-90, 0, -90, 90, -90, 0];
a     = [47, 110, 26, 0, 0, 0]*1e-3;
d     = [133, 0, 0, 117.5, 0, 28]*1e-3;   
theta = [theta1, theta2-90,theta3,theta4,theta5,theta6];   
theta


%% Calculate Transformation Matrix for each joint
for i=1:length(theta)
   T(:,:,i) = DHMatrix(alpha(i),a(i),d(i),theta(i));
end


% Calculate Forward Kinematics and Simulation
T_end = eye(4,4);
for k=1:length(theta)
    temp = T_end;
    T_end = T_end* T(:,:,k);
    % Simulation
    plot3([temp(1,4) T_end(1,4)],[temp(2,4) T_end(2,4)],[temp(3,4) T_end(3,4)],'k','LineWidth',5);
    hold on,grid on,axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);
    scatter3(T_end(1,4),T_end(2,4),T_end(3,4),'MarkerFaceColor',[1 0 0]);
    scatter3(0,0,0,'MarkerFaceColor',[1 0 0]); 
    xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('6DOF Robot manipulator simulation')  
end
str1 = {['X : ',num2str(T_end(1,4)),' Y : ',num2str(T_end(2,4)),' Z : ',num2str(T_end(3,4))]};
str2 = {['X : ',num2str(0),' Y : ',num2str(0),' Z : ',num2str(0)]};
text(T_end(1,4)+.010,T_end(2,4),T_end(3,4),str1,'Color','k','FontSize',10)
text(0,0,-.010,str2,'Color','k','FontSize',10)
save M T_end a d alpha
run IK


% END Effector Orientation Convert 
Rcorr = [inv(T_end(1:3,1:3)) zeros(3,1); 0 0 0 1];
T_endCorr = Rcorr*T_end;

% END Effector Orientation Convert 
Rcorr = [inv(T_end(1:3,1:3)) zeros(3,1); 0 0 0 1];
T_endCorr = T_end * Rcorr;

% Plot Base Frame
plot3([0 .2],[0 0],[0 0],'r--','LineWidth',1.5)
plot3([0 0],[0 .2],[0 0],'g--','LineWidth',1.5)
plot3([0 0],[0 0],[0 .2],'b--','LineWidth',1.5)




