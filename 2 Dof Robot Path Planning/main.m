clc,clear all,close all,warning off;
%% 2 Dof Robotic Arm Path Planning (Joint Space) and Ýnverse Kinematics solve
% Written By Raþit EVDÜZEN
% 11-Jun-2016

% Robot % Path Parameter
l1 = 4;   % Robot link 1
l2 = 2;   % Robot link 2
ll = l1+l2; % total link length
t1 = [];    %
t2 = [];    %
time = 2;   % each joint target position ( initial and final) path generation and complate time
t = 0:0.05:time;

% Target Position and velocity
% p   = [-6 2;-3 -1;2 -2;6 0; 4 3; 2 5; 4 4];      % Robot set position
p   = [6 0; 4 3; 2 5; 4 4];      % Robot set position
% [td0 tdf]
vel1 = [0 5;5 5;5 0];   % joint 1 velocity
vel2 = [0 5;5 5;5 0];   % joint 2 velocity
[R,C] = size(p);
cplot = @(r,x0,y0) plot(x0 + r*cos(linspace(0,2*pi)),y0 + r*sin(linspace(0,2*pi)),'-');

% Find the teta1 and teta2 with inverse kinematics function for joint space
% polinoms parameter
for k = 1:R
    xhat = p(k,1);
    yhat = p(k,2);
    [teta1,teta2] = ik(xhat,yhat,l1,l2);
    t1 = [t1,teta1];
    t2 = [t2,teta2];
end

for i = 1:length(t1)-1
    %     [P1,Pd1,Pdd1] = pth(t1(i),t1(i+1),vel1(i,1),vel1(i,2),time);
    %     [P2,Pd2,Pdd2] = pth(t2(i),t2(i+1),vel2(i,1),vel2(i,2),time);
    [P1,Pd1,Pdd1] = pth(t1(i),t1(i+1),0,0,time);
    [P2,Pd2,Pdd2] = pth(t2(i),t2(i+1),0,0,time);
    
    subplot(221)
    hold off
    %         scatter(j,Pd1(j),'filled','k','LineWidth',0.1)
    plot(t,P1,'b:','LineWidth',1.5)
    hold on,grid on
    plot(t,Pd1,'k:','LineWidth',1.5)
    plot(t,Pdd1,'r:','LineWidth',1.5)
    xlabel('Sn') , ylabel('Degree / Sn'), title('First Link Velocity')
    legend('Angular Position','Angular Velocity','Angular Acceleratiion')
    
    subplot(222)
    hold off
    plot(t,P1,'b:','LineWidth',1.5)
    hold on,grid on
    plot(t,Pd2,'k--','LineWidth',1.5)
    plot(t,Pdd2,'r:','LineWidth',1.5)
    xlabel('Sn') , ylabel('Degree / Sn'), title('Second Link Velocity')
    legend('Angular Position','Angular Velocity','Angular Acceleratiion')
    % Robot Plot
    for j=1:length(P1)
        
        set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
        A1x = [0 l1*cos(deg2rad(P1(j)))];
        A1y = [0 l1*sin(deg2rad(P1(j)))];
        A2x = [A1x(:,2) A1x(:,2)+l2*cos(deg2rad(P1(j)+P2(j)))];
        A2y = [A1y(:,2) A1y(:,2)+l2*sin(deg2rad(P1(j)+P2(j)))];
        
        subplot(2,2,[3,4])
        hold off
        plot(A1x,A1y,'r','LineWidth',2)
        hold on,grid on
        cplot(ll,0,0)
        plot(A2x,A2y,'b','LineWidth',2)
        axis equal
        axis([-ll-1 ll+1 -ll-1 ll+1])
        xlabel('X - Axis') , ylabel('Y - Axis'), title('2 Dof Robot Arm')
        scatter(p(1,1),p(1,2),'Filled','k')
        scatter(p(2,1),p(2,2),'Filled','k')
        scatter(p(3,1),p(3,2),'Filled','k')
        scatter(p(4,1),p(4,2),'Filled','k')
%         scatter(p(5,1),p(5,2),'Filled','k')
%         scatter(p(6,1),p(6,2),'Filled','k')
%         scatter(p(7,1),p(7,2),'Filled','k')
        
        pause(0.0000001)
    end
    
end
