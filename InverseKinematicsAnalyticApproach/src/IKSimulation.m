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
torchGeo = stlread("torch.STL");

%---------------------- Robot DH Parameters ----------------------%
a2 = 175; a3 = 890; a4 = 50;
d1 = 575; d4 = 1035; d6 = 185;
a     = [0,a2,a3,a4,0,0];
alpha = [0,90, 0, 90, -90, 90];
d     = [d1, 0, 0, d4, 0, d6];
DhParam = [a; alpha; d];
%-----------------------------------------------------------------%
% Welding Torch Option
WTorch = "On";  % Welding Torch On ~ Off
WTx = -1.8;  % X offset [mm] Real Value = 1.80248
WTy = -1.16; % Y offset [mm] Real Value = -1.15921
WTz = 436.99;   % Z offset [mm] Real Value = 436.99162
WTRPY = [0 -50 0]*pi/180;  % Welding Torch Roll Pitch Yaw [0 -50 0]
WTtransMat = [eul2rotm(WTRPY,'ZYX') [WTx WTy WTz]'; 0 0 0 1];
iWTtransMat = [eul2rotm(WTRPY,'ZYX')' eul2rotm(WTRPY,'ZYX')'*-[WTx WTy WTz]'; 0 0 0 1];
% WTtransMat*iWTtransMat  % Identity Matrix Check
% %-----------------------------------------------------------------%
% Type of Trajectory
% 0 -> Home Pose
% 1 -> 3D Circle & Line
% 2 -> 3D Line
% 3 -> 3D Cube 
% 4 -> Star
% 5 -> Inf
% 6 -> Wrist Dance
% 7 -> Linear Welding Traj, dont forget the set [WTorch = "On"]
% 8 -> Sinusoidal Welding Traj, dont forget the set [WTorch = "On"]


TrajSelect = 0;   % 0 Home Pose
NoD = 50;        % Number of sample
Traj = WorkSpaceTraj(TrajSelect,NoD);
Roll = 0;
Pitch = -45;
Yaw = -180;
eul = [Roll Pitch Yaw]*pi/180;
eeOrient = eul2rotm(eul,'ZYX');  % Default "ZYX" For Home Pose R=0,P=-90,Y=-180
%-----------------------------------------------------------------%

%-----------------------------------------------------------------%

AllTargetPose = zeros(4,4,size(Traj,1));
AllMatrix = zeros(4,24,size(Traj,1));
for i = 1:size(Traj,1)
    if TrajSelect == 6 % Wrist Dance
        Roll = linspace(0,90,NoD);
        Pitch = linspace(0,0,NoD);
        Yaw = linspace(90,90,NoD);
        eul = [Roll(i) Pitch(i) Yaw(i)]*pi/180;
        eeOrient = eul2rotm(eul,'ZYX');  
    end
    AllTargetPose(:,:,i) = [eeOrient [Traj(i,1) Traj(i,2) Traj(i,3)]' ; 0 0 0 1];
end

%-----------------------------------------------------------------%
Ts = 1e-2;       % Sampling Periode
%------------------ Calculate Inverse Kinematics -----------------%
for k=1:size(Traj,1)-1
    if WTorch == "On"
        AllTargetPose(:,:,k) = AllTargetPose(:,:,k)*iWTtransMat;  % Finding Robot EE Pose
    end
    iversethetaInitial = IK(AllTargetPose(:,:,k),DhParam);
    iversethetaFinal   = IK(AllTargetPose(:,:,k+1),DhParam);
    %-------------------- Trajectory Generation ----------------------%
    SimTheta = zeros(1,6);
    for j=1:length(iversethetaFinal)
        SimTheta(j) = JointSpaceTraj(iversethetaInitial(j), iversethetaFinal(j), Ts,0,1);
    end
    %--------- ROBOT Forward Kinematics Calculate Matrix -----------%
    T06 = eye(4,4);
    tmp = [];
    T = zeros(4,4,6);
    
    if TrajSelect == 0 & WTorch ~= "On"
        SimTheta = [0 90 0 0 0 0]; % Park Pose Angle
    end

    for i=1:size(SimTheta,2)
        T(:,:,i) = DHMatrixModify(alpha(i),a(i),d(i),SimTheta(i));
        T06 = T06 * T(:,:,i);
        tmp = [tmp T06];
    end
    AllMatrix(:,:,k) = tmp;
end

%% Robot Simulation
figure('units','normalized','outerposition',[0 0 1 1],'color','w')
for i = 1:size(AllMatrix,3)-1
    clf
    plot3(Traj(:,1),Traj(:,2),Traj(:,3),'k',LineWidth=2),hold on,grid on
    xlabel('X-axis [mm]'),ylabel('Y-axis [mm]'),zlabel('Z-axis [mm]')
    title('KR70 Robot Inverse Kinematics Simulation')
    axis equal,axis([-2500 2500 -2500 2500 0 2500])
    view(45,20)  % First arguman is azimuth angle, second is elevation angle
    PlotRobot(AllMatrix(:,:,i),baseGeo,link1Geo,link2Geo,link3Geo,link4Geo,link5Geo,link6Geo,torchGeo,WTorch,WTtransMat,TrajSelect)
    drawnow
end

