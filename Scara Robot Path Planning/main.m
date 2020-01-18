clc,clear all,close all,warning off;
%% Scara Robotic Arm Path Planning (Joint Space) and Ýnverse Kinematics solve
% Written By Raþit EVDÜZEN
% 16-Jun-2016

% Robot % Path Parameter
base  = 5;  % Robot Base
l1 = 8;     % Robot link 1
l2 = 8;     % Robot link 2
ll = l1+l2; % total link length
t1 = [];    %
t2 = [];    %
t3 = [];    %
time = 2;   % each joint target position ( initial and final) path generation and complate time
t = 0:0.05:time;

% Target Position and velocity (x,y,z)
p   = [-6 2 1;-3 -1 2;2 -2 3;6 0 2; 4 3 1; 2 5 4; 4 4 0];      % Robot set position
% p   = [2 -2 1;6 0 2;4 3 3];      % Robot set position
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
    zhat = p(k,3);
    [teta1,teta2,d] = ik(xhat,yhat,zhat,l1,l2);
    t1 = [t1,teta1];
    t2 = [t2,teta2];
    t3 = [t3,d];
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
    plot(t,P2,'b:','LineWidth',1.5)
    hold on,grid on
    plot(t,Pd2,'k--','LineWidth',1.5)
    plot(t,Pdd2,'r:','LineWidth',1.5)
    xlabel('Sn') , ylabel('Degree / Sn'), title('Second Link Velocity')
    legend('Angular Position','Angular Velocity','Angular Acceleratiion')
    % Robot Plot
    for j=1:length(P1)
        
        set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
        A0x = [0 0];
        A0y = [0 0];
        A0z = [0 base];
        A1x = [0 l1*cos(deg2rad(P1(j)))];
        A1y = [0 l1*sin(deg2rad(P1(j)))];
        A1z = [base base];
        A2x = [A1x(:,2) A1x(:,2)+l2*cos(deg2rad(P1(j)+P2(j)))];
        A2y = [A1y(:,2) A1y(:,2)+l2*sin(deg2rad(P1(j)+P2(j)))];
        A2z = [base base];
        A3x = [A2x(:,2) A2x(:,2)];
        A3y = [A2y(:,2) A2y(:,2)];
        A3z = [base t3(i)];  % z ekseni icin yörünge planlamasýný kodla daha
        % daha sonra kodlarý düzelt kinematiðini incele
        
        subplot(2,2,[3,4])
        hold off
        plot3(A0x,A0y,A0z,'k:','LineWidth',2)
        hold on,grid minor
        plot3(A1x,A1y,A1z,'r','LineWidth',2)
        cplot(ll,0,0)
        plot3(A2x,A2y,A2z,'b','LineWidth',2)
        plot3(A3x,A3y,A3z,'g','LineWidth',2)
        
        
        axis equal
        axis([-ll-1 ll+1 -ll-1 ll+1])
        xlabel('X - Axis') , ylabel('Y - Axis'),zlabel('Z - Axis'), title('Scara Robot')
        scatter3(p(1,1),p(1,2),p(1,3),'Filled','k')
        scatter3(p(2,1),p(2,2),p(2,3),'Filled','k')
        scatter3(p(3,1),p(3,2),p(3,3),'Filled','k')
        scatter3(p(4,1),p(4,2),p(4,3),'Filled','k')
        scatter3(p(5,1),p(5,2),p(5,3),'Filled','k')
        scatter3(p(6,1),p(6,2),p(6,3),'Filled','k')
        scatter3(p(7,1),p(7,2),p(7,3),'Filled','k')
        
        pause(0.0000001)
    end
    
end
