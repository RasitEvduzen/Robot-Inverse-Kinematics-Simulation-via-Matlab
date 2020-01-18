%% Calculate Inverse Kinematics
load M
% Finding Wrist Center (Kinematic Decoupling)
Xsw = [T_end(1:3,4)] - [d(6) * T_end(1:3,3)];

% InverseTheta - 1
itheta1 = [atan2(Xsw(2),Xsw(1))] - [atan2(d(3),sqrt(Xsw(1)^2 + Xsw(2)^2 + d(3)^2))];
ci1 = cos(itheta1);
si1 = sin(itheta1);
itheta1 = [(itheta1*180/pi)];


% InverseTheta - 2
itheta2 = (pi/2) - [acos((a(2)^2+(Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-a(1))^2-(a(3)^2+d(4)^2))/(2*a(2)*sqrt((Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-a(1))^2)))] - [atan((Xsw(3)-d(1))/(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-a(1)))];
ci2 = cos(itheta2);
si2 = sin(itheta2);
itheta2 = (itheta2*180/pi);


% InverseTheta - 3
itheta3 = (pi)-[acos((a(2)^2+a(3)^2+d(4)^2-(Xsw(3)-d(1))^2-(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-a(1))^2)/(2*a(2)*sqrt(a(3)^2+d(4)^2)))] - [atan(d(4)/a(3))];
ci3 = cos(itheta3);
si3 = sin(itheta3);
itheta3 = (itheta3*180/pi);


% Calculate 4 - 5 - 6 Angle
T_01 = DHMatrix(alpha(1),a(1),d(1),theta(1));
T_12 = DHMatrix(alpha(2),a(2),d(2),theta(2));
T_23 = DHMatrix(alpha(3),a(3),d(3),theta(3));
T_03 = T_01 * T_12 * T_23;
T_06 = T_end;   % End effector Target Position &  Orientation
T_36 = inv(T_03) * T_06;


% InverseTheta - 4
itheta4 = atan2(-T_36(2,3),-T_36(1,3));
ci4 = cos(itheta4);
si4 = sin(itheta4);
itheta4 = (itheta4*180/pi);


% InverseTheta - 5
itheta5 = atan2(sqrt(T_36(1,3)^2 + T_36(2,3)^2),T_36(3,3));
ci5 = cos(itheta5);
si5 = sin(itheta5);
itheta5 = (itheta5*180/pi);


% InverseTheta - 6
itheta6 = atan2(-T_36(3,2),T_36(3,1));
ci6 = cos(itheta6);
si6 = sin(itheta6);
itheta6 = (itheta6*180/pi);

itheta = [itheta1, itheta2-90, itheta3, itheta4, itheta5, itheta6];   


%% Calculate Transformation Matrix for each joint
for i=1:length(theta)
   iT(:,:,i) = DHMatrix(alpha(i),a(i),d(i),itheta(i));
end

% Calculate Forward Kinematics and Simulation
iT_end = eye(4,4);
for k=1:length(theta)
    temp = iT_end;
    iT_end = iT_end* iT(:,:,k);
    % Simulation
    plot3([temp(1,4) iT_end(1,4)],[temp(2,4) iT_end(2,4)],[temp(3,4) iT_end(3,4)],'r--','LineWidth',3);
    scatter3(iT_end(1,4),iT_end(2,4),iT_end(3,4),'MarkerFaceColor',[1 0 0])
    scatter3(0,0,0,'MarkerFaceColor',[1 0 0]); 
    xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('6DOF Robot manipulator simulation')  
end
itheta