clc;clear;close all;warning off;
%% 6 DOF ROBOTIC ARM FORWARD & INVERSE KINEMATIC ANALYSIS via Numerical Optimization
% Kuka KR70 ROBOTIC ARM
% Written By: Rasit EVDUZEN
% 24-Mar-2024

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
InitialTheta = [0 90 0 0 0 0];             % Initial Angle For Optimization
Theta = InitialTheta';
% ForwardKinematics(Theta,DhParam)
%---------------------- Robot Target Pose & Orientation ----------------------%
% Home Pose (X,Y,Z,R,P,Y) -> [1395,0,1515,0,90,-180]
TpX = 1250;    
TpY = 1000;       
TpZ = 1300;  
TpRoll = 0;    
TpPitch = 0;   
TpYaw = -90;    
eul = [TpRoll TpPitch TpYaw];
TargetPoint = [TpX TpY TpZ eul];
eeOrient = eul2rotm(deg2rad(eul),'XYZ');  % Default "ZYX" For Home Pose R=0,P=-90,Y=-180
TargetPose = [eeOrient [TpX TpY TpZ]';0 0 0 1];  % TargetPose 4x4 Homogeneous Transformation Matrix

% trplot(eye(4),'frame','0','thick',1,'rgb','length',300),hold on
% trplot(TargetPose,'frame','TargetPose','thick',1,'rgb','length',500)
% title('KUKA KR70 IK Solution via Steepest Descent')
% axis equal,axis([-2500 2500 -2500 2500 0 2500])
% view(145,20)  % First arguman is azimuth angle, second is elevation angle
% return

s = -2e-5;  % Step Size
NoIter = 1e6;
StopCondition = 1e-3;
SimTheta = zeros(NoIter,6);
for k=1:NoIter
    %------------------ Calculate Inverse Kinematics -----------------%
    [X,Y,Z,Roll,Pitch,Yaw] = ForwardKinematics(Theta,DhParam);  % Calculate FK value
    y = [X,Y,Z,Roll,Pitch,Yaw]';
    e = [TargetPoint'- y];            % Calculate Error
    Jr = Jacobian(Theta,DhParam);            % Calculate Jacobian  There are error !
    Theta = Theta - s*Jr'*e; % Update Parameter via Gradient direction
    SimTheta(k,:) = Theta';
    if mod(k,1e3) == 0
        disp("||Error||"+num2str(norm(e)))
    end
    if abs(norm(e)) <= StopCondition
        SimTheta = SimTheta(1:k,:);
        break
    end
end

AllMatrix = zeros(4,24,size(SimTheta,1));
for j=1:size(SimTheta,1)
    %--------- ROBOT Forward Kinematics Calculate Matrix -----------%
    T06 = eye(4,4);
    tmp = [];
    T = zeros(4,4,6);
    for i=1:size(SimTheta,2)
        T(:,:,i) = DHMatrixModify(alpha(i),a(i),d(i),SimTheta(j,i));
        T06 = T06 * T(:,:,i);
        tmp = [tmp T06];
    end
    AllMatrix(:,:,j) = tmp;
end

%% Robot Simulation
figure('units','normalized','outerposition',[0 0 1 1],'color','w')
DispMatrixSize = size(AllMatrix,3)-mod(size(AllMatrix,3),1e4);
for i = 1:DispMatrixSize
    clf
    xlabel('X-axis [mm]'),ylabel('Y-axis [mm]'),zlabel('Z-axis [mm]'),hold on,grid on
    trplot(TargetPose,'frame','TargetPose','thick',1,'rgb','length',500)
    title('KUKA KR70 IK Solution via Steepest Descent')
    axis equal,axis([-2500 2500 -2500 2500 0 2500])
    view(45,20)  % First arguman is azimuth angle, second is elevation angle
    if i < 20
        PlotRobot(AllMatrix(:,:,i),baseGeo,link1Geo,link2Geo,link3Geo,link4Geo,link5Geo,link6Geo)
        idx = i;
    else 
        idx = idx + 1e4;
        PlotRobot(AllMatrix(:,:,idx),baseGeo,link1Geo,link2Geo,link3Geo,link4Geo,link5Geo,link6Geo)
    end
    if idx > DispMatrixSize
        break
    end
    drawnow
end

