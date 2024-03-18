function [s, sd, sdd] = JointSpaceTraj(q0,q1,t,v,Ts)

timeVec = 0:Ts:t;
m = (q1 - q0) / (t - 0);
s = [m * (timeVec) + q0]';
% sd = [v * ones(length(timeVec),1)];
% sdd = [zeros(length(timeVec),1)];