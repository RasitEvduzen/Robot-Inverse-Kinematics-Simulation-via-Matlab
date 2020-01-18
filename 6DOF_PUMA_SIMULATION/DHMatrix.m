function [Output] = DHMatrix(alpha,a,d,theta)

alpha = alpha*pi/180;    % Deg to Rad
theta = theta*pi/180;    % Deg to Rad
c = cos(theta);
s = sin(theta);
ca = cos(alpha);
sa = sin(alpha);
Output = [     c   -s   0     a; 
            s*ca c*ca -sa -sa*d;
            s*sa c*sa  ca  ca*d;
               0    0   0     1];


end
       