clc,clear all,close all,warning off;
%% 6 DOF ROBOTIC ARM Computed Torque Control
% Kuka KR70 ROBOTIC ARM
% Written By: Rasit EVDUZEN
% 26-Apr-2020
%% ROBOT SIMULATION

%---------------------- Robot DH Parameters ----------------------%
a2 = 175; a3 = 890; a4 = 50;
d1 = 575; d4 = 1035; d6 = 185;
a     = [0,a2,a3,a4,0,0];
alpha = [0,90, 0, 90, -90, 90];
d     = [d1, 0, 0, d4, 0, d6];
DhParam = [a; alpha; d];
%-----------------------------------------------------------------%
% Type of Trajectory P2P
X = 1495;
Y = 00;
Z = 1515;
Roll = 0;     % A
Pitch = -90;  % B
Yaw = -180;   % C
eul = [Roll Pitch Yaw]*pi/180;
eeOrient = eul2rotm(eul,'ZYX');  % Default "ZYX" For Home Pose R=0,P=-90,Y=-180
%-----------------------------------------------------------------%
HomeTransMat = [0 0 1 1395; 0 -1 0 0; 1 0 0 1515; 0 0 0 1];
AllTargetPose(:,:,1) = HomeTransMat;
AllTargetPose(:,:,2) = [eeOrient [X Y Z]'; 0 0 0 1];

%------------------ Calculate Inverse Kinematics -----------------%
iversethetaInitial = IK(AllTargetPose(:,:,1),DhParam);
iversethetaFinal   = IK(AllTargetPose(:,:,2),DhParam);

% -------------------------- Traj Param --------------------------
Ts = 1e-3;       % Sampling Periode
% ------------ [180 158 160 230 230 320]    % KR70 Default Axis Speed
MaxVelocity = [180 158 160 230 230 320];   % Kuka KR70 Joint Angular Speed

AngleDifference = abs(iversethetaFinal - iversethetaInitial);
SpeedAngleTime = abs(AngleDifference./MaxVelocity);

%--------- Smooth Jerk------------
TrajTime = 2*max(SpeedAngleTime);  % Trajectory Time [Sn] For Max Speed  Tb = 1e-2
TbPercent = .25;          % -%50 Acceleration
tSimulink = (0:Ts:TrajTime)';

Tblend = TrajTime*TbPercent;             % Each Axis Blend Time
Ta = TrajTime - (2 * Tblend);
SimAngularVel = abs(iversethetaFinal - iversethetaInitial)./(Tblend + Ta);

%-------------------- Trajectory Generation ----------------------%
SimTheta = [];   % Angular Pose
SimDTheta = [];  % Angular Velocity
SimDDTheta = []; % Angular Acceleration
for j=1:length(iversethetaFinal)
    [q,qdot,qddot] =  SCurveTrajectory(iversethetaInitial(j), iversethetaFinal(j), TrajTime, SimAngularVel(j),Ts);
    SimTheta   = [SimTheta q];
    SimDTheta  = [SimDTheta qdot];
    SimDDTheta = [SimDDTheta qddot];
end

