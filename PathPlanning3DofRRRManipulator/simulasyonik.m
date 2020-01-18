clc,clear all,close all;
%% Master - Robot Kinematics Final
% Manipülatör 3DOF (RRR) Ýnverse Kinematics and Path Planning Simulation
% Pamukkale Universty EEM MSC
% Written By: Raþit EVDÜZEN
% 20-Dec-2019 

figure
set(gcf,'Position',[100 100 1720 900])

% Robot Parameters for manipulator
l1 = 300; % Robot Link Configration
l2 = 400;
l3 = 400;
% Target position & oriantation
x = [500,-100,300,300,-100,-100,500];
y = [0,400,400,400,400,400,0];
z = [300,600,600,200,200,600,300];
t1 = []; t2 = []; t3 = [];
for i = 1:length(x)
    clf
    [t1(i),t2(i),t3(i)] = ik(x(i),y(i),z(i),l1,l2,l3);
    
    % Robot DH parameters
    alpha = [0, 90, 0, 0];
    a     = [0, 0, l2, l3];
    d     = [l1, 0, 0, 0];
    theta = [t1(i), t2(i), t3(i), 0];
    
    % Calculate Transformation Matrix for each joint
    for i=1:length(theta)
        T(:,:,i) = DHMatrix(alpha(i),a(i),d(i),theta(i));
    end
    
    % Calculate Forward Kinematics and Simulation
    T_end = eye(4,4);
    for k=1:length(theta)
        temp = T_end;
        T_end = T_end* T(:,:,k);   % Calculate forward kinematics via DH method
        % Simulation
        plot3([temp(1,4) T_end(1,4)],[temp(2,4) T_end(2,4)],[temp(3,4) T_end(3,4)],'k','LineWidth',3);
        hold on,grid on,axis([-550 550 -550 550 -100 1000]);
        scatter3(T_end(1,4),T_end(2,4),T_end(3,4),'MarkerFaceColor',[0 1 0]);
        scatter3(0,0,0,'MarkerFaceColor',[0 1 0]);
        xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('3 DOF RRR Robot manipulator simulation')
        
    end
    for j = 1:length(x)-1
        plot3([x(j) x(j+1)],[y(j) y(j+1)],[z(j) z(j+1)],'b','LineWidth',2),hold on
    end
    v = [-40 20 10];
    view(v)
    str1 = {['X : ',num2str(T_end(1,4)),' Y : ',num2str(T_end(2,4)),' Z : ',num2str(T_end(3,4))]};
    str2 = {['X : ',num2str(0),' Y : ',num2str(0),' Z : ',num2str(0)]};
    text(T_end(1,4)+10,T_end(2,4),T_end(3,4),str1,'Color','k','FontSize',10)
    text(0,0,-10,str2,'Color','k','FontSize',10)
    pause(0.5)
end
% return
%% Plot each joint position & velocity & acceleration
% joint - 1 - 2 - 3
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
    
    timevec = linspace(0,5,length(P1));
    subplot(322)
    plot(timevec,P1,'r','LineWidth',2), hold on, grid minor
    plot(timevec,Pd1,'g','LineWidth',1.5)
    plot(timevec,Pdd1,'b--','LineWidth',1)
    title('Joint - 1 Position & Velocity & Acceleration')
    legend('pos','vel','acc')
    axis([-1 6.5 -30 110])
    xlabel('Time')

    subplot(324)
    % Normalization graph 
%     P2 = [P2 - min(P2)] / [max(P2) - min(P2)];
%     Pd2 = [Pd2 - min(Pd2)] / [max(Pd2) - min(Pd2)];
%     Pdd2 = [Pdd2 - min(Pdd2)] / [max(Pdd2) - min(Pdd2)];
    plot(timevec,P2,'r','LineWidth',2), hold on, grid minor
    plot(timevec,Pd2,'g','LineWidth',1.5)
    plot(timevec,Pdd2,'b--','LineWidth',1)
    title('Joint - 2 Position & Velocity & Acceleration')
    legend('pos','vel','acc')
    axis([-1 6.5 -20 100])
    xlabel('Time')
    
    subplot(326)
    plot(timevec,P3,'r','LineWidth',2), hold on, grid minor
    plot(timevec,Pd3,'g','LineWidth',1.5)
    plot(timevec,Pdd3,'b--','LineWidth',1)
    title('Joint - 3 Position & Velocity & Acceleration')
    legend('pos','vel','acc')
    axis([-1 6.5 -150 10])
    xlabel('Time')
    
    % Calculate End Effector Position and orientation
    angle = [P1;P2;P3;zeros(1,length(P1))]';
    X = []; Y = []; Z = [];
    for q=1:length(P1)
        for i=1:4
            T(:,:,i) = DHMatrix(alpha(i),a(i),d(i),angle(q,i));
        end
        T_end1 = eye(4);
        for i =1:4
            T_end1 = T_end1 * T(:,:,i);
        end
        X = [X,T_end1(1,4)];
        Y = [Y,T_end1(2,4)];
        Z = [Z,T_end1(3,4)];
    end
    
    subplot(321)
    plot(timevec,X,'k','LineWidth',2), grid minor
    title('End Effector X position (mm)')
    xlabel('Time'),ylabel('End Effector X (mm) position')
    
    subplot(323)
    plot(timevec,Y,'k','LineWidth',2), grid minor
    title('End Effector Y position (mm)')
    xlabel('Time'),ylabel('End Effector Y (mm) position')
    
    subplot(325)
    plot(timevec,Z,'k','LineWidth',2), grid minor
    title('End Effector Z position (mm)')
    xlabel('Time'),ylabel('End Effector Z (mm) position')
    
    pause(0.5)
end






