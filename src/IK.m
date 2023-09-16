function [JointAngle] = IK(TEnd,DhParam)
%---------------------------------------------------------------------------
% *******************************
% Input Parameters: 
% *TEnd    = Robot Target Pose
% *DhParam = Robot DH Parameters
% *******************************
% Output Parameters:
% JointAngle = Inverse Kinematics Solution
% There are 8 solution, please be careful how to solution need
% *******************************
% SR-80 DH Parameters
% a2 = 175; a3 = 905; a4 = 206; 
% d1 = 630; d4 = 1020; d6 = 205;
% a     = [0,a2,a3,a4,0,0];
% alpha = [0,90, 0, 90, -90, 90];
% d     = [d1, 0, 0, d4, 0, d6];
% DhParam = [a; alpha; d];
% *******************************
% Written By: Rasit EVDUZEN
% Date      : 23-Dec-2021
% Version   : V1
% *******************************
%---------------------------------------------------------------------------
a     = DhParam(1,:);
alpha = DhParam(2,:);
d     = DhParam(3,:);

%---------------------------------------------------------------------------
% Calculate Joint1
N06 = TEnd(1:3,3);
P46 = d(6) * N06;
P06 = TEnd(1:3,4);
P04 = P06 - P46; % Wrist Center (Kinematic Decoupling)

% There are two solution for Joint 1
J1_1 = atan2(P04(2),P04(1));
J1_2 = atan2(P04(2),P04(1)) + pi;
% J1_3 = atan2(P04(2),P04(1)) - pi;
J1deg = [J1_1 J1_2]*180/pi;  % Two Solution!!!!!
T01 = DHMatrixModify(alpha(1),a(1),d(1),J1deg(1)); % Solution Selection !!

%---------------------------------------------------------------------------
% Calculate Joint3
T12 = DHMatrixModify(alpha(2),a(2),d(2),0); % theta2 = 0 for solution
TempT02 = T01 * T12;
P02 = TempT02(1:3,4);
P24 = P04 - P02;
% P24 = P04 - [a(2)*cos(J1_1) a(2)*sin(J1_1) d(1)]'; % Another Solution P24
L1 = sqrt(a(4)^2 + d(4)^2);
Gamma = atan2(d(4),a(4));    % WARNING!!!
% phi = asin((L1^2 - a(3)^2 + norm(P24)^2)/(2*norm(P24)*L1)) + asin(((norm(P24))-((L1^2 - a(3)^2 + norm(P24)^2)/(2*norm(P24))))/(a(3)));
% J3_1 = -pi + phi + Gamma; % Wrong Solution
% J3_2 = -(-pi + phi - Gamma);  % Correct Solution
% J3deg = [J3_1 J3_2]*180/pi

Phi = acos((( a(3)^2 + (L1^2) - norm(P24)^2) )  / (2 * a(3) * L1));
n = -1; %  or -1
J3_1 =  -n*[pi - (Phi - Gamma)];  % Wrong Solution
J3_2 =  n*[pi - (Phi + Gamma)];  % Correct Solution
J3deg = [J3_1 J3_2]*180/pi; %
T23 = DHMatrixModify(alpha(3),a(3),d(3),J3deg(2)); % Solution Selection !!

%---------------------------------------------------------------------------
% Calculate Joint2
P24_1 = inv(T01(1:3,1:3)) * P24;
Beta1 = atan2(P24_1(3),P24_1(1));
Beta2 = acos(( a(3)^2+norm(P24)^2-L1^2)/(2*a(3)*norm(P24)));
n = 1;
J2_1 = n*(Beta1 + Beta2);
J2_2 = n*(Beta1 - Beta2);
J2deg = [J2_1 J2_2]*180/pi; % two possible solution
T12 = DHMatrixModify(alpha(2),a(2),d(2),J2deg(1)); % Solution Selection !!

%---------------------------------------------------------------------------
% Calculate Joint5
T34 = DHMatrix(alpha(4),a(4),d(4),0);
T04 = T01 * T12 * T23 * T34;

J5_1 = acos(dot(T04(1:3,3),TEnd(1:3,3)));
J5deg = [J5_1]*180/pi;  % Correct

%---------------------------------------------------------------------------
% Calculate Joint4
R46 = inv(T04(1:3,1:3)) * TEnd(1:3,1:3);
J4_1 = atan2(R46(2,3), R46(1,3));
J4deg = [J4_1]*180/pi;

%---------------------------------------------------------------------------
% % Calculate Joint6
J6_1 = atan2(R46(3,2) , -R46(3,1));
J6deg = [J6_1]*180/pi;

JointAngle = [J1deg(1) J2deg(1) J3deg(2) J4deg J5deg J6deg];
end

