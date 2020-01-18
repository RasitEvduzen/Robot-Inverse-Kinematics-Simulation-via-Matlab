clc,clear all,close all;
%% PUMA - 762 FORWARD & INVERSE KÝNEMATÝC ANALYSÝS
% Written By: Raþit EVDÜZEN
% 23-Dec-2019 
% Version: 0
figure
set(gcf,'Position',[100 100 1720 900])
N = 10;  % Number Of Iteration

t1 = []; t2 = []; t3 = []; t4 = []; t5 = []; t6 = [];   % For Path Planning
for q =1:N
    clf
%% ROBOT CONFÝGRATÝON 1
% Joint Parameters
theta1 = 90*rand();    
theta2 = 90*rand();
theta3 = 90*rand();
theta4 = 90*rand();
theta5 = 90*rand();
theta6 = 90*rand();
         
% Robot DH parameters
%         a     = link length
%         alpha = link twist
%         th    = joint angle
%         d     = link offset
a2 = 650; 
a3 = 0; 
d3 = 190; 
d4 = 600;
d6 = 0; 
% % Robot DH parameters 
alpha = [0, -90, 0, -90, 90, -90,0];
a     = [0, 0, a2, a3, 0, 0, 0];
d     = [0, 0, d3, d4, 0, 0, d6];
theta = [theta1+90, theta2-90,theta3-90,theta4,theta5,theta6,0];      


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
    plot3([temp(1,4) T_end(1,4)],[temp(2,4) T_end(2,4)],[temp(3,4) T_end(3,4)],'b','LineWidth',3);
    hold on,grid on,axis([-3500 3500 -3500 3500 -3500 3500]);
    scatter3(T_end(1,4),T_end(2,4),T_end(3,4),'MarkerFaceColor',[1 0 0]);
    scatter3(0,0,0,'MarkerFaceColor',[1 0 0]); 
    xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis')  
end
str1 = {['X : ',num2str(T_end(1,4)),' Y : ',num2str(T_end(2,4)),' Z : ',num2str(T_end(3,4))]};
str2 = {['X : ',num2str(0),' Y : ',num2str(0),' Z : ',num2str(0)]};
text(T_end(1,4)+10,T_end(2,4),T_end(3,4),str1,'Color','k','FontSize',10)
text(0,0,-10,str2,'Color','k','FontSize',10)


%% INVERSE KINEMATIC SOLUTION 
T_end1 = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7);        % End Effector Position and Orientation
[itheta1, itheta2, itheta3, itheta4, itheta5, itheta6] = invKinematics(T_end1);
iftheta = [itheta1, itheta2, itheta3, itheta4, itheta5, itheta6];
ftheta = theta;
ftheta = ftheta(1,1:6);
ftheta
iftheta
t1(q) = itheta1; t2(q) = itheta2; t3(q) = itheta3; t4(q) = itheta4; t5(q) = itheta5; t6(q) = itheta6;
% PLOT INVERSE KINEMATIC MODEL 
    
%% Calculate Transformation Matrix for each joint
iftheta(1,7) = 0;
for i=1:length(theta)
   Ti(:,:,i) = DHMatrix(alpha(i),a(i),d(i),iftheta(i));
end
% Calculate Forward Kinematics Via Inverse Solution
T_end = eye(4,4);
for k=1:length(theta)
    temp = T_end;
    T_end = T_end* Ti(:,:,k);
    plot3([temp(1,4) T_end(1,4)],[temp(2,4) T_end(2,4)],[temp(3,4) T_end(3,4)],'r--','LineWidth',3);
    hold on,grid on
    scatter3(T_end(1,4),T_end(2,4),T_end(3,4),'MarkerFaceColor',[1 0 0]);
    scatter3(0,0,0,'MarkerFaceColor',[1 0 0]); 
end
title({'PUMA 762 6DOF Robot manipulator simulation';'Blue -> Forward Solution  Red -> Inverse Solution'})

pause(0.1)
end


%% Plot each joint position & velocity & acceleration
% joint - 1 - 2 - 3 - 4 - 5 - 6
figure
set(gcf,'Position',[100 100 1720 900])
for l = 1:length(t1)-1
    clf
    initialVel = 0;
    finalVel = 0;
    time = 5;
    [P1,Pd1,Pdd1] = pth(t1(l),t1(l+1),initialVel,finalVel,time);
    [P2,Pd2,Pdd2] = pth(t2(l),t2(l+1),initialVel,finalVel,time);
    [P3,Pd3,Pdd3] = pth(t3(l),t3(l+1),initialVel,finalVel,time);
    [P4,Pd4,Pdd4] = pth(t3(l),t3(l+1),initialVel,finalVel,time);
    [P5,Pd5,Pdd5] = pth(t3(l),t3(l+1),initialVel,finalVel,time);
    [P6,Pd6,Pdd6] = pth(t3(l),t3(l+1),initialVel,finalVel,time);
    timevec = linspace(0,5,length(P1));
    
    subplot(321)
    plot(timevec,P1,'r','LineWidth',2), hold on, grid minor
    plot(timevec,Pd1,'g','LineWidth',1.5)
    plot(timevec,Pdd1,'b--','LineWidth',1)
    title('Joint - 1 Position & Velocity & Acceleration')
    legend('pos','vel','acc')
%     axis([-1 6.5 -100 200])
    xlabel('Time')

    subplot(322)
    plot(timevec,P2,'r','LineWidth',2), hold on, grid minor
    plot(timevec,Pd2,'g','LineWidth',1.5)
    plot(timevec,Pdd2,'b--','LineWidth',1)
    title('Joint - 2 Position & Velocity & Acceleration')
    legend('pos','vel','acc')
%     axis([-1 6.5 -100 200])
    xlabel('Time')
    
    subplot(323)
    plot(timevec,P3,'r','LineWidth',2), hold on, grid minor
    plot(timevec,Pd3,'g','LineWidth',1.5)
    plot(timevec,Pdd3,'b--','LineWidth',1)
    title('Joint - 3 Position & Velocity & Acceleration')
    legend('pos','vel','acc')
%     axis([-1 6.5 -100 200])
    xlabel('Time')
    
    subplot(324)
    plot(timevec,P4,'r','LineWidth',2), hold on, grid minor
    plot(timevec,Pd4,'g','LineWidth',1.5)
    plot(timevec,Pdd4,'b--','LineWidth',1)
    title('Joint - 4 Position & Velocity & Acceleration')
    legend('pos','vel','acc')
%     axis([-1 6.5 -100 200])
    xlabel('Time')

    subplot(325)
    plot(timevec,P5,'r','LineWidth',2), hold on, grid minor
    plot(timevec,Pd5,'g','LineWidth',1.5)
    plot(timevec,Pdd5,'b--','LineWidth',1)
    title('Joint - 5 Position & Velocity & Acceleration')
    legend('pos','vel','acc')
%     axis([-1 6.5 -100 200 ])
    xlabel('Time')
    
    subplot(326)
    plot(timevec,P6,'r','LineWidth',2), hold on, grid minor
    plot(timevec,Pd6,'g','LineWidth',1.5)
    plot(timevec,Pdd6,'b--','LineWidth',1)
    title('Joint - 6 Position & Velocity & Acceleration')
    legend('pos','vel','acc')
%     axis([-1 6.5 -100 200])
    xlabel('Time')
    
    pause(0.1)
end
