clc,clear all,close all,warning off;
%% 6 DOF ROBOTIC ARM FORWARD & INVERSE KINEMATIC ANALYSIS
% Kuka KR70 ROBOTIC ARM
% Written By: Rasit EVDUZEN
% 26-Apr-2020

%% ROBOT SIMULATION
%---------------------- Load Robot Geometry ----------------------%
addpath('stl')
baseGeo = stlread("base.STL");
link1Geo = stlread("link1.STL");
link2Geo = stlread("link2.STL");
link3Geo = stlread("link3.STL");
link4Geo = stlread("link4.STL");
link5Geo = stlread("link5.STL");
link6Geo = stlread("link6.STL");

%---------------------- Robot DH Parameters ----------------------%
a2 = 175; a3 = 890; a4 = 50;
d1 = 575; d4 = 1035; d6 = 185;
a     = [0,a2,a3,a4,0,0];
alpha = [0,90, 0, 90, -90, 90];
d     = [d1, 0, 0, d4, 0, d6];
DhParam = [a; alpha; d];
%-----------------------------------------------------------------%
% Type of Trajectory P2P
X = 1000;
Y = 1500;
Z = 1250;
Roll = 0;     % A
Pitch = 90;  % B
Yaw = 0;   % C
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
Ts = 1e-2;       % Sampling Periode
% ------------ [180 158 160 230 230 320]    % KR70 Default Axis Speed
MaxVelocity = [180 158 160 230 230 320];   % Kuka KR70 Joint Angular Speed

AngleDifference = abs(iversethetaFinal - iversethetaInitial);
SpeedAngleTime = abs(AngleDifference./MaxVelocity);

%--------- Smooth Jerk------------
TrajTime = 2*max(SpeedAngleTime);  % Trajectory Time [Sn] For Max Speed  Tb = 1e-2
TbPercent = .5;          % -%50 Acceleration

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
%--------- ROBOT Forward Kinematics Calculate Matrix -----------%

AllMatrix = zeros(4,24,size(SimTheta,1));
TrSimTime = linspace(0,TrajTime,size(SimTheta,1));
EETraj = zeros(size(SimTheta,1),3);
for k=1:size(SimTheta,1)
    T06 = eye(4,4);
    tmp = [];
    T = zeros(4,4,6);
    for i=1:size(SimTheta,2)
        T(:,:,i) = DHMatrixModify(alpha(i),a(i),d(i),SimTheta(k,i));
        T06 = T06 * T(:,:,i);
        tmp = [tmp T06];
    end
    EETraj(k,:) = T06(1:3,4);
    AllMatrix(:,:,k) = tmp;
end

%% Robot Simulation
figure('units','normalized','outerposition',[0 0 1 1],'color','w')
gif('RobotTraj.gif')
for i = 1:size(AllMatrix,3)
    if mod(i,2) == 0
        clf
        plot3(EETraj(:,1),EETraj(:,2),EETraj(:,3),'k',LineWidth=2),hold on,grid on
        xlabel('X-axis [mm]'),ylabel('Y-axis [mm]'),zlabel('Z-axis [mm]')
        title('KR70 Robot Joint Space Trapezoidal Trajectory Planning')
        axis equal,axis([-2500 2500 -2500 2500 0 2500])
        trplot(AllTargetPose(:,:,2),'frame','TP','thick',1,'rgb','length',500)
        view(65,15)  % First arguman is azimuth angle, second is elevation angle
        PlotRobot(AllMatrix(:,:,i),baseGeo,link1Geo,link2Geo,link3Geo,link4Geo,link5Geo,link6Geo)
        drawnow
        gif
    end
end

figure('units','normalized','outerposition',[0 0 1 1],'color','w')
PlotRobotJoint(TrSimTime,SimTheta,SimDTheta,SimDDTheta)

