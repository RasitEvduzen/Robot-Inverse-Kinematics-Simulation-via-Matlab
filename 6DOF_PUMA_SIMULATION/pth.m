function [P,Pd,Pdd] = pth(t0,tf,td0,tdf,time)
% Written by Raþit EVDÜZEN
% 11-Jun-2019 

% 3 order polinoms for path generator
% find the coeffient
% this parameters include initial and final velocity if dont need initial
% and final veclocity than s2 and s3 coeffient rewrite
s0 = t0;
s1 = td0;
s2 = (3/time^2) * (tf - t0) - (2/time)*td0 - (1/time)*tdf;
s3 = (-2/time^3) * (tf - t0) + (1/time^2) * (td0 + tdf);

% P   => position equation
% Pd  => velocity equation
% Pdd => acceleration equation
P = []; Pd = []; Pdd = [];
for t = 0:0.05:time
    P   = [P,(s0) + (s1 * t) + (s2 * t.^2) + (s3 * t.^3)];  % calculate position polinoms
    Pd  = [Pd,(s1) + (2 * s2 * t) + (3 * s3 * t.^2)];       % calculate velocity polinoms
    Pdd = [Pdd,(2 * s2) + (6 * s3 * t)];                    % calculate acceleration polinoms
end

end

