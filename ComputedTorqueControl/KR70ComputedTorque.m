clc,clear all,close all,warning off;
%% 6 DOF ROBOTIC ARM Computed Torque Control
% Kuka KR70 ROBOTIC ARM
% Written By: Rasit EVDUZEN
% 27-Apr-2024
%% ROBOT SIMULATION
% -------------------- Robot Dynamic Parameters ------------------%
DynamicParam
%---------------------- Robot DH Parameters ----------------------%
a2 = 175; a3 = 890; a4 = 50;
d1 = 575; d4 = 1035; d6 = 185;
a     = [0,a2,a3,a4,0,0];
alpha = [0,90, 0, 90, -90, 90];
d     = [d1, 0, 0, d4, 0, d6];
JointOffset = [0 90 0 0 0 0];
DhParam = [a; alpha; d];
%-----------------------------------------------------------------%
% Type of Trajectory P2P
X = 1200;
Y = 1200;
Z = 1200;
Roll = 45;     % A
Pitch = 45;  % B
Yaw =  45;   % C
eul = [Roll Pitch Yaw]*pi/180;
eeOrient = eul2rotm(eul,'ZYX');  % Default "ZYX" For Home Pose X=1395, Y=0, Z=1515 R=0,P=-90,Y=-180
%-----------------------------------------------------------------%
load AllTargetPose.mat

SimTheta = [];   % Angular Pose
SimDTheta = [];  % Angular Velocity
SimDDTheta = []; % Angular Acceleration
for i=1:size(AllTargetPose,3)-1
%------------------ Calculate Inverse Kinematics -----------------%
iversethetaInitial = IK(AllTargetPose(:,:,i),DhParam);
iversethetaInitial = iversethetaInitial - JointOffset;
iversethetaFinal   = IK(AllTargetPose(:,:,i+1),DhParam);
iversethetaFinal = iversethetaFinal - JointOffset;
% -------------------------- Traj Param --------------------------
Ts = 1e-3;       % Sampling Periode
% ------------ [180 158 160 230 230 320]    % KR70 Default Axis Speed
MaxVelocity = [180 158 160 230 230 320];   % Kuka KR70 Joint Angular Speed

AngleDifference = abs(iversethetaFinal - iversethetaInitial);
SpeedAngleTime = abs(AngleDifference./MaxVelocity);

%--------- Smooth Jerk------------
TrajTime = 2*max(SpeedAngleTime);  % Trajectory Time [Sn] For Max Speed  Tb = 1e-2
TbPercent = .25;          % 50 Acceleration

Tblend = TrajTime*TbPercent;             % Each Axis Blend Time
Ta = TrajTime - (2 * Tblend);
SimAngularVel = abs(iversethetaFinal - iversethetaInitial)./(Tblend + Ta);

%-------------------- Trajectory Generation ----------------------%
TmpTheta = [];   
TmpDTheta = [];   
TmpDDTheta = [];
EESim = [];
for j=1:length(iversethetaFinal)
    [q,qdot,qddot] =  SCurveTrajectory(iversethetaInitial(j), iversethetaFinal(j), TrajTime, SimAngularVel(j),Ts);
    TmpTheta   = [TmpTheta q];
    TmpDTheta  = [TmpDTheta qdot];
    TmpDDTheta = [TmpDDTheta qddot];
end
SimTheta = [SimTheta;TmpTheta];
SimDTheta = [SimDTheta;TmpDTheta];   
SimDDTheta = [SimDDTheta;TmpDDTheta];   
end

%----------- ROBOT Forward Kinematics Data -------------%
SimThetaPoint = SimTheta + JointOffset;
EEPoseSim = zeros(size(SimThetaPoint,1),3);
EEOrientSim = zeros(size(SimThetaPoint,1),3);
for k=1:size(SimThetaPoint,1)
    T06 = eye(4,4);
    for i=1:size(SimThetaPoint,2)
        temp = T06;
        T(:,:,i) = DHMatrixModify(alpha(i),a(i),d(i),SimThetaPoint(k,i));
        T06 = T06 * T(:,:,i);       
    end
    EEPoseSim(k,:) = [T06(1,4) T06(2,4) T06(3,4)];
    EEOrientSim(k,:) = rad2deg(rotm2eul(T06(1:3,1:3),'ZYX'));
end
SimulinkSimTime = size(SimThetaPoint,1)*Ts;
tSimulink = linspace(0,size(SimThetaPoint,1)*Ts,size(SimThetaPoint,1))';


% Controller Parameters
Kp = [1e3 1e3 1e3 3e3 3e3 1e6];
Ki = [3e3 1e3 1e3 1e2 1e2 1e3];
Kv = [1e2 1e2 1e2 1e2 1e2 1e3];

out = sim("KUKAKR70.slx");

%% Plot Result Joint Space Traj
figure('units','normalized','outerposition',[0 0 1 1],'color','w')

for t=1:6
subplot(2,3,t)
plot(tSimulink,deg2rad(SimTheta(:,t)),'r',LineWidth=1)
hold on, grid on
plot(tSimulink,out.JointAngle.Data(1:end-1,t),'k',LineWidth=1)
ylabel('Angle', 'Fontsize', 12);
xlabel('t (second)')
title("Joint: "+num2str(t)+'  S-Curve Pose Profile')
legend('$\theta(t) (Rad) Referance$','$\dot{\theta}(t) (Rad) Real$','Interpreter','latex','fontsize',12)
legend('boxoff')
axis auto
end

figure('units','normalized','outerposition',[0 0 1 1],'color','w')
for t=1:6
subplot(2,3,t)
plot(tSimulink(20:end),deg2rad(SimDTheta(20:end,t)),'r',LineWidth=1)
hold on, grid on
plot(tSimulink(20:end),out.JointVel.Data(20:end-1,t),'k',LineWidth=1)
ylabel('Angular Velocity', 'Fontsize', 12);
xlabel('t (second)')
title("Joint: "+num2str(t)+'  S-Curve Velocity Profile')
legend('$\dot{\theta}(t) (Rad/Sn) Referance$','$\dot{\theta}(t) (Rad/Sn) Real$','Interpreter','latex','fontsize',12)
legend('boxoff')
axis auto
end

%% Plot Result Joint Torque
figure('units','normalized','outerposition',[0 0 1 1],'color','w')
for t=1:6
subplot(2,3,t)
hold on, grid on
plot(tSimulink(50:end),out.JointTorque.Data(50:end-1,t),'k',LineWidth=1)
ylabel('Torque [Nm]', 'Fontsize', 12);
xlabel('t (second)')
title("Joint: "+num2str(t)+'  Torque')
legend('Torque Val','fontsize',12)
legend('boxoff')
axis auto
end


