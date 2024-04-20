clc,clear,close all,warning off;
%% 6 DOF ROBOTIC ARM Inverse Dynamics Analysis
% Kuka KR70 ROBOTIC ARM
% Written By: Rasit EVDUZEN
% 20-Apr-2024
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
X = 1200;
Y = 1200;
Z = 1200;
Roll = 45;     % A
Pitch = 45;  % B
Yaw =  45;   % C
eul = [Roll Pitch Yaw]*pi/180;
eeOrient = eul2rotm(eul,'ZYX');  % Default "ZYX" For Home Pose X=1395, Y=0, Z=1515 R=0,P=-90,Y=-180
%-----------------------------------------------------------------%
% AllTargetPose(:,:,1) = [0 0 1 1395; 0 -1 0 0; 1 0 0 1515; 0 0 0 1];  % Default "ZYX" For Home Pose X=1395, Y=0, Z=1515 R=0,P=-90,Y=-180
% AllTargetPose(:,:,2) = [eeOrient [X Y Z]'; 0 0 0 1];
load AllTargetPose.mat

SimTheta = [];   % Angular Pose
SimDTheta = [];  % Angular Velocity
SimDDTheta = []; % Angular Acceleration
for i=1:size(AllTargetPose,3)-1
%------------------ Calculate Inverse Kinematics -----------------%
iversethetaInitial = IK(AllTargetPose(:,:,i),DhParam);
iversethetaFinal   = IK(AllTargetPose(:,:,i+1),DhParam);
% -------------------------- Traj Param --------------------------
Ts = 1e-3;       % Sampling Periode
% ------------ [180 158 160 230 230 320]    % KR70 Default Axis Speed
MaxVelocity = [180 158 160 230 230 320];   % Kuka KR70 Joint Angular Speed

AngleDifference = abs(iversethetaFinal - iversethetaInitial);
SpeedAngleTime = abs(AngleDifference./MaxVelocity);

%--------- Smooth Jerk------------
TrajTime = 2*max(SpeedAngleTime);  % Trajectory Time [Sn] For Max Speed  Tb = 1e-2
TbPercent = .25;          % -%50 Acceleration

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
EEPoseSim = zeros(size(SimTheta,1),3);
EEOrientSim = zeros(size(SimTheta,1),3);
for k=1:size(SimTheta,1)
    T06 = eye(4,4);
    for i=1:size(SimTheta,2)
        temp = T06;
        T(:,:,i) = DHMatrixModify(alpha(i),a(i),d(i),SimTheta(k,i));
        T06 = T06 * T(:,:,i);       
    end
    EEPoseSim(k,:) = [T06(1,4) T06(2,4) T06(3,4)];
    EEOrientSim(k,:) = rad2deg(rotm2eul(T06(1:3,1:3),'ZYX'));
end
SimulinkSimTime = size(SimTheta,1)*Ts;
tSimulink = linspace(0,size(SimTheta,1)*Ts,size(SimTheta,1))';

sim("KUKAKR70.slx");
% ---------------------------- Plot Simulation Result ----------------------------
%% Plot Joint Space Trajectory Profile
figure('units','normalized','outerposition',[0 0 1 1],'color','w')

subplot(321)
plot(tSimulink,deg2rad(SimTheta(:,1)),'r')
hold on, grid minor
plot(tSimulink,deg2rad(SimDTheta(:,1)),'g')
plot(tSimulink,deg2rad(SimDDTheta(:,1)),'b')
ylabel('Joint 1', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Rad)$','$\dot{\theta}(t) (Rad/Sn)$','$\ddot{\theta}(t) (Rad/Sn^2)$','Interpreter','latex','fontsize',12)
legend('boxoff')
axis auto

subplot(323)
plot(tSimulink,deg2rad(SimTheta(:,2)),'r')
hold on, grid minor
plot(tSimulink,deg2rad(SimDTheta(:,2)),'g')
plot(tSimulink,deg2rad(SimDDTheta(:,2)),'b')
ylabel('Joint 2', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Rad)$','$\dot{\theta}(t) (Rad/Sn)$','$\ddot{\theta}(t) (Rad/Sn^2)$','Interpreter','latex','fontsize',12)
legend('boxoff')
axis auto

subplot(325)
plot(tSimulink,deg2rad(SimTheta(:,3)),'r')
hold on, grid minor
plot(tSimulink,deg2rad(SimDTheta(:,3)),'g')
plot(tSimulink,deg2rad(SimDDTheta(:,3)),'b')
ylabel('Joint 3', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Rad)$','$\dot{\theta}(t) (Rad/Sn)$','$\ddot{\theta}(t) (Rad/Sn^2)$','Interpreter','latex','fontsize',12)
legend('boxoff')
axis auto

subplot(322)
plot(tSimulink,deg2rad(SimTheta(:,4)),'r')
hold on, grid minor
plot(tSimulink,deg2rad(SimDTheta(:,4)),'g')
plot(tSimulink,deg2rad(SimDDTheta(:,4)),'b')
ylabel('Joint 4', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Rad)$','$\dot{\theta}(t) (Rad/Sn)$','$\ddot{\theta}(t) (Rad/Sn^2)$','Interpreter','latex','fontsize',12)
legend('boxoff')
axis auto

subplot(324)
plot(tSimulink,deg2rad(SimTheta(:,5)),'r')
hold on, grid minor
plot(tSimulink,deg2rad(SimDTheta(:,5)),'g')
plot(tSimulink,deg2rad(SimDDTheta(:,5)),'b')
ylabel('Joint 5', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Rad)$','$\dot{\theta}(t) (Rad/Sn)$','$\ddot{\theta}(t) (Rad/Sn^2)$','Interpreter','latex','fontsize',12)
legend('boxoff')
axis auto

subplot(326)
plot(tSimulink,deg2rad(SimTheta(:,6)),'r')
hold on, grid minor
plot(tSimulink,deg2rad(SimDTheta(:,6)),'g')
plot(tSimulink,deg2rad(SimDDTheta(:,6)),'b')
ylabel('Joint 6', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Rad)$','$\dot{\theta}(t) (Rad/Sn)$','$\ddot{\theta}(t) (Rad/Sn^2)$','Interpreter','latex','fontsize',12)
legend('boxoff')
axis auto
%% End Effector Pose Trajectory XYZ ABC
figure('units','normalized','outerposition',[0 0 1 1],'color','w')

subplot(321)
plot(tSimulink,EEPoseSim(:,1)),grid on
xlabel('Time'), ylabel('Mm'),title('End Effector X Trajectory')

subplot(323)
plot(tSimulink,EEPoseSim(:,2)),grid on
xlabel('Time'), ylabel('Mm'),title('End Effector Y Trajectory')

subplot(325)
plot(tSimulink,EEPoseSim(:,3)),grid on
xlabel('Time'), ylabel('Mm'),title('End Effector Z Trajectory')

subplot(322)
plot(tSimulink,EEOrientSim(:,1)),grid on
xlabel('Time'), ylabel('Deg'),title('End Effector Roll X-Axis')

subplot(324)
plot(tSimulink,EEOrientSim(:,2)),grid on
xlabel('Time'), ylabel('Deg'),title('End Effector Pitch Y-Axis')

subplot(326)
plot(tSimulink,EEOrientSim(:,3)),grid on
xlabel('Time'), ylabel('Deg'),title('End Effector Yaw Z-Axis')

%% Plot Simulation Torque Variable
tSimulink = ans.tout;  % Simulink Simulation Time Update!
figure('units','normalized','outerposition',[0 0 1 1],'color','w')

% Joint 1 
J1Torque = ans.J1Trq.Data;
[J1Rms,J1Max,J1Min,J1Mean,J1Std,J1Diff] = GetStat(tSimulink,J1Torque);

subplot(321)
plot(tSimulink,J1Torque),grid minor
xlabel('Time (sn)')
ylabel("J1 Torque [Nm]")
J1Str = ['Max: ',num2str(J1Max), ' Min: ', num2str(J1Min),' Mean: ',num2str(J1Mean),' Std: ',num2str(J1Std),  ' Rms: ',num2str(J1Rms)];
title(J1Str)

% Joint 2 
J2Torque = ans.J2Trq.Data;
[J2Rms,J2Max,J2Min,J2Mean,J2Std,J2Diff] = GetStat(tSimulink,J2Torque);

subplot(323)
plot(tSimulink,J2Torque),grid minor
xlabel('Time (sn)')
ylabel("J2 Torque [Nm]")
J2Str = ['Max: ',num2str(J2Max), ' Min: ', num2str(J2Min),' Mean: ',num2str(J2Mean),' Std: ',num2str(J2Std),  ' Rms: ',num2str(J2Rms)];
title(J2Str)

% Joint 3 
J3Torque = ans.J3Trq.Data;
[J3Rms,J3Max,J3Min,J3Mean,J3Std,J3Diff] = GetStat(tSimulink,J3Torque);

subplot(325)
plot(tSimulink,J3Torque),grid minor
xlabel('Time (sn)')
ylabel("J3 Torque [Nm]")
J3Str = ['Max: ',num2str(J3Max), ' Min: ', num2str(J3Min),' Mean: ',num2str(J3Mean),' Std: ',num2str(J3Std),  ' Rms: ',num2str(J3Rms)];
title(J3Str)

% Joint 4 
J4Torque = ans.J4Trq.Data;
[J4Rms,J4Max,J4Min,J4Mean,J4Std,J4Diff] = GetStat(tSimulink,J4Torque);

subplot(322)
plot(tSimulink,J4Torque),grid minor
xlabel('Time (sn)')
ylabel("J4 Torque [Nm]")
J4Str = ['Max: ',num2str(J4Max), ' Min: ', num2str(J4Min),' Mean: ',num2str(J4Mean),' Std: ',num2str(J4Std),  ' Rms: ',num2str(J4Rms)];
title(J4Str)

% Joint 5
J5Torque = ans.J5Trq.Data;
[J5Rms,J5Max,J5Min,J5Mean,J5Std,J5Diff] = GetStat(tSimulink,J5Torque);

subplot(324)
plot(tSimulink,J5Torque),grid minor
xlabel('Time (sn)')
ylabel("J5 Torque [Nm]")
J5Str = ['Max: ',num2str(J5Max), ' Min: ', num2str(J5Min),' Mean: ',num2str(J5Mean),' Std: ',num2str(J5Std),  ' Rms: ',num2str(J5Rms)];
title(J5Str)

% Joint 6
J6Torque = ans.J6Trq.Data;
[J6Rms,J6Max,J6Min,J6Mean,J6Std,J6Diff] = GetStat(tSimulink,J6Torque);

subplot(326)
plot(tSimulink,J6Torque), grid minor
xlabel('Time (sn)')
ylabel("J6 Torque [Nm]")
J6Str = ['Max: ',num2str(J6Max), ' Min: ', num2str(J6Min),' Mean: ',num2str(J6Mean),' Std: ',num2str(J6Std),  ' Rms: ',num2str(J6Rms)];
title(J6Str)


%% Plot Each Axis Force & Torque

figure('units','normalized','outerposition',[0 0 1 1],'color','w')

% Joint 1 Torque 
J1CTorque = ans.J1Torque.Data;
J1RaialTorque = sqrt(J1CTorque(:,1).^2+J1CTorque(:,2).^2); % Resultant


subplot(321)
plot(tSimulink,J1CTorque),grid minor, hold on
plot(tSimulink,J1RaialTorque,'k--')
xlabel('Time (sn)')
ylabel("J1 Torque [Nm]")
title("J1 3-Axis Torque")
legend("Mx (Radial Torque)","My (Radial Torque)","Mz (Axial Torque)",'Resultant Radial Torque')

% Joint 1 Force 
J1Force = ans.J1Force.Data;
J1RaialForce = sqrt(J1Force(:,1).^2+J1Force(:,2).^2); % Resultant

subplot(322)
plot(tSimulink,J1Force),grid minor, hold on
plot(tSimulink,J1RaialForce,'k--')
xlabel('Time (sn)')
ylabel("J1 Force [N]")
title("J1 3-Axis Force")
legend("Fx (Radial Force)","Fy (Radial Force)","Fz (Axial Force)",'Resultant Radial Force')

% Joint 2 Torque 
J2CTorque = ans.J2Torque.Data;
J2RaialTorque = sqrt(J2CTorque(:,1).^2+J2CTorque(:,2).^2); % Resultant

subplot(323)
plot(tSimulink,J2CTorque),grid minor, hold on
plot(tSimulink,J2RaialTorque,'k--')
xlabel('Time (sn)')
ylabel("J2 Torque [Nm]")
title("J2 3-Axis Torque")
legend("Mx (Radial Torque)","My (Radial Torque)","Mz (Axial Torque)",'Resultant Radial Torque')

% Joint 2 Force 
J2Force = ans.J2Force.Data;
J2RaialForce = sqrt(J2Force(:,1).^2+J2Force(:,2).^2); % Resultant

subplot(324)
plot(tSimulink,J2Force),grid minor, hold on
plot(tSimulink,J2RaialForce,'k--')
xlabel('Time (sn)')
ylabel("J2 Force [N]")
title("J2 3-Axis Force")
legend("Fx (Radial Force)","Fy (Radial Force)","Fz (Axial Force)",'Resultant Radial Force')

% Joint 3 Torque 
J3CTorque = ans.J3Torque.Data;
J3RaialTorque = sqrt(J3CTorque(:,1).^2+J3CTorque(:,2).^2); % Resultant

subplot(325)
plot(tSimulink,J3CTorque),grid minor, hold on
plot(tSimulink,J3RaialTorque,'k--')
xlabel('Time (sn)')
ylabel("J3 Torque [Nm]")
title("J3 3-Axis Torque")
legend("Mx (Radial Torque)","My (Radial Torque)","Mz (Axial Torque)",'Resultant Radial Torque')

% Joint 3 Force 
J3Force = ans.J3Force.Data;
J3RaialForce = sqrt(J3Force(:,1).^2+J3Force(:,2).^2); % Resultant

subplot(326)
plot(tSimulink,J3Force),grid minor, hold on
plot(tSimulink,J3RaialForce,'k--')
xlabel('Time (sn)')
ylabel("J3 Force [N]")
title("J3 3-Axis Force")
legend("Fx (Radial Force)","Fy (Radial Force)","Fz (Axial Force)",'Resultant Radial Force')

figure('units','normalized','outerposition',[0 0 1 1],'color','w')

% Joint 4 Torque 
J4CTorque = ans.J4Torque.Data;
J4RaialTorque = sqrt(J4CTorque(:,1).^2+J4CTorque(:,2).^2); % Resultant

subplot(321)
plot(tSimulink,J4CTorque),grid minor, hold on
plot(tSimulink,J4RaialTorque,'k--')
xlabel('Time (sn)')
ylabel("J4 Torque [Nm]")
title("J4 3-Axis Torque")
legend("Mx (Radial Torque)","My (Radial Torque)","Mz (Axial Torque)",'Resultant Radial Torque')

% Joint 4 Force 
J4Force = ans.J4Force.Data;
J4RaialForce = sqrt(J4Force(:,1).^2+J4Force(:,2).^2); % Resultant

subplot(322)
plot(tSimulink,J4Force),grid minor, hold on
plot(tSimulink,J4RaialForce,'k--')
xlabel('Time (sn)')
ylabel("J4 Force [N]")
title("J4 3-Axis Force")
legend("Fx (Radial Force)","Fy (Radial Force)","Fz (Axial Force)",'Resultant Radial Force')

% Joint 5 Torque 
J5CTorque = ans.J5Torque.Data;
J5RaialTorque = sqrt(J5CTorque(:,1).^2+J5CTorque(:,2).^2); % Resultant

subplot(323)
plot(tSimulink,J5CTorque),grid minor, hold on
plot(tSimulink,J5RaialTorque,'k--')
xlabel('Time (sn)')
ylabel("J5 Torque [Nm]")
title("J5 3-Axis Torque")
legend("Mx (Radial Torque)","My (Radial Torque)","Mz (Axial Torque)",'Resultant Radial Torque')

% Joint 5 Force 
J5Force = ans.J5Force.Data;
J5RaialForce = sqrt(J5Force(:,1).^2+J5Force(:,2).^2); % Resultant

subplot(324)
plot(tSimulink,J5Force),grid minor, hold on
plot(tSimulink,J5RaialForce,'k--')
xlabel('Time (sn)')
ylabel("J5 Force [N]")
title("J5 3-Axis Force")
legend("Fx (Radial Force)","Fy (Radial Force)","Fz (Axial Force)",'Resultant Radial Force')

% Joint 6 Torque 
J6CTorque = ans.J6Torque.Data;
J6RaialTorque = sqrt(J6CTorque(:,1).^2+J6CTorque(:,2).^2); % Resultant

subplot(325)
plot(tSimulink,J6CTorque),grid minor, hold on
plot(tSimulink,J6RaialTorque,'k--')
xlabel('Time (sn)')
ylabel("J6 Torque [Nm]")
title("J6 3-Axis Torque")
legend("Mx (Radial Torque)","My (Radial Torque)","Mz (Axial Torque)",'Resultant Radial Torque')

% Joint 6 Force 
J6Force = ans.J6Force.Data;
J6RaialForce = sqrt(J6Force(:,1).^2+J6Force(:,2).^2); % Resultant

subplot(326)
plot(tSimulink,J6Force),grid minor, hold on
plot(tSimulink,J6RaialForce,'k--')
xlabel('Time (sn)')
ylabel("J6 Force [N]")
title("J6 3-Axis Force")
legend("Fx (Radial Force)","Fy (Radial Force)","Fz (Axial Force)",'Resultant Radial Force')

%% Plot Each Axis Inertia
figure('units','normalized','outerposition',[0 0 1 1],'color','w')

% Joint 1 Inertia 
J1Inertia = ans.J1Inertia.Data;
J1Izz = J1Inertia(3,3,:);
J1Izz = reshape(J1Izz,1,size(tSimulink,1));

subplot(321)
plot(tSimulink,J1Izz),grid minor, hold on
yline(max(J1Izz),'r')
xlabel('Time (sn)')
ylabel("[KgM^2]")
Str2 = ["J1 Inertia Max: ",num2str(max(J1Izz))];
title(Str2)
legend("Izz")

% Joint 2 Inertia 
J2Inertia = ans.J2Inertia.Data;
J2Izz = J2Inertia(3,3,:);
J2Izz = reshape(J2Izz,1,size(tSimulink,1));

subplot(322)
plot(tSimulink,J2Izz),grid minor, hold on
yline(max(J2Izz),'r')
xlabel('Time (sn)')
ylabel("[KgM^2]")
Str2 = ["J2 Inertia Max: ",num2str(max(J2Izz))];
title(Str2)
legend("Izz")

% Joint 3 Inertia 
J3Inertia = ans.J3Inertia.Data;
J3Izz = J3Inertia(3,3,:);
J3Izz = reshape(J3Izz,1,size(tSimulink,1));

subplot(323)
plot(tSimulink,J3Izz),grid minor, hold on
yline(max(J3Izz),'r')
xlabel('Time (sn)')
ylabel("[KgM^2]")
Str2 = ["J3 Inertia Max: ",num2str(max(J3Izz))];
title(Str2)
legend("Izz")

% Joint 4 Inertia 
J4Inertia = ans.J4Inertia.Data;
J4Izz = J4Inertia(3,3,:);
J4Izz = reshape(J4Izz,1,size(tSimulink,1));

subplot(324)
plot(tSimulink,J4Izz),grid minor, hold on
yline(max(J4Izz),'r')
xlabel('Time (sn)')
ylabel("[KgM^2]")
Str2 = ["J4 Inertia Max: ",num2str(max(J4Izz))];
title(Str2)
legend("Izz")

% Joint 5 Inertia 
J5Inertia = ans.J5Inertia.Data;
J5Izz = J5Inertia(3,3,:);
J5Izz = reshape(J5Izz,1,size(tSimulink,1));

subplot(325)
plot(tSimulink,J5Izz),grid minor, hold on
yline(max(J5Izz),'r')
xlabel('Time (sn)')
ylabel("[KgM^2]")
Str2 = ["J5 Inertia Max: ",num2str(max(J5Izz))];
title(Str2)
legend("Izz")

% Joint 6 Inertia 
J6Inertia = ans.J6Inertia.Data;
J6Izz = J6Inertia(3,3,:);
J6Izz = reshape(J6Izz,1,size(tSimulink,1));

subplot(326)
plot(tSimulink,J6Izz),grid minor, hold on
yline(max(J6Izz),'r')
xlabel('Time (sn)')
ylabel("[KgM^2]")
Str2 = ["J6 Inertia Max: ",num2str(max(J6Izz))];
title(Str2)
legend("Izz")

%% Plot EE Linear&Angular Velocity
figure('units','normalized','outerposition',[0 0 1 1],'color','w')

% EE X-Axis Linear Velocity
EEXVel = ans.EEXVel.Data;

subplot(321)
plot(tSimulink,EEXVel),grid minor, hold on
xlabel('Time (sn)')
ylabel("[M/Sn]")
Str2 = ["End Effector Linear Velocity Max: ",num2str(max(EEXVel))];
title(Str2)
legend("X-Axis")

% EE Y-Axis Linear Velocity
EEYVel = ans.EEYVel.Data;

subplot(323)
plot(tSimulink,EEYVel),grid minor, hold on
xlabel('Time (sn)')
ylabel("[M/Sn]")
Str2 = ["End Effector Linear Velocity Max: ",num2str(max(EEYVel))];
title(Str2)
legend("Y-Axis")

% EE Z-Axis Linear Velocity
EEZVel = ans.EEZVel.Data;

subplot(325)
plot(tSimulink,EEZVel),grid minor, hold on
xlabel('Time (sn)')
ylabel("[M/Sn]")
Str2 = ["End Effector Linear Velocity Max: ",num2str(max(EEZVel))];
title(Str2)
legend("Z-Axis")

% EE X-Axis Angular Velocity
EEWxVel = ans.EEWxVel.Data;

subplot(322)
plot(tSimulink,EEWxVel),grid minor, hold on
xlabel('Time (sn)')
ylabel("[Rad/Sn]")
Str2 = ["End Effector Angular Velocity Max: ",num2str(max(EEWxVel))];
title(Str2)
legend("X-Axis")

% EE Y-Axis Angular Velocity
EEWyVel = ans.EEWyVel.Data;

subplot(324)
plot(tSimulink,EEWyVel),grid minor, hold on
xlabel('Time (sn)')
ylabel("[Rad/Sn]")
Str2 = ["End Effector Angular Velocity Max: ",num2str(max(EEWyVel))];
title(Str2)
legend("Y-Axis")

% EE Z-Axis Angular Velocity
EEWzVel = ans.EEWzVel.Data;

subplot(326)
plot(tSimulink,EEWzVel),grid minor, hold on
xlabel('Time (sn)')
ylabel("[Rad/Sn]")
Str2 = ["End Effector Angular Velocity Max: ",num2str(max(EEWzVel))];
title(Str2)
legend("Z-Axis")


%% Turn off return for create simulation result via csv file
return
%% Data Output

% J1 TrqFrc
J1FrcTrq = [J1CTorque J1RaialTorque J1Force J1RaialForce];
J1FrcTrq = num2cell(J1FrcTrq);
J1FrcTrq(1,:) = {'J1Mx (Nm)', 'J1My (Nm)', 'J1Mz (Nm)', 'J1RadialTorq (Nm)', 'J1Fx (N)', 'J1Fy (N)', 'J1Fz (N)', 'J1RadialForce (N)'};
writetable(cell2table(J1FrcTrq),'SimOutData\TrqFrc\J1TrqFrc.csv')

% J2 TrqFrc
J2FrcTrq = [J2CTorque J2RaialTorque J2Force J2RaialForce];
J2FrcTrq = num2cell(J2FrcTrq);
J2FrcTrq(1,:) = {'J2Mx (Nm)', 'J2My (Nm)', 'J2Mz (Nm)', 'J2RadialTorq (Nm)', 'J2Fx (N)', 'J2Fy (N)', 'J2Fz (N)', 'J2RadialForce (N)'};
writetable(cell2table(J2FrcTrq),'SimOutData\TrqFrc\J2TrqFrc.csv')

% J3 TrqFrc
J3FrcTrq = [J3CTorque J3RaialTorque J3Force J3RaialForce];
J3FrcTrq = num2cell(J3FrcTrq);
J3FrcTrq(1,:) = {'J3Mx (Nm)', 'J3My (Nm)', 'J3Mz (Nm)', 'J3RadialTorq (Nm)', 'J3Fx (N)', 'J3Fy (N)', 'J3Fz (N)', 'J3RadialForce (N)'};
writetable(cell2table(J3FrcTrq),'SimOutData\TrqFrc\J3TrqFrc.csv')

% J4 TrqFrc
J4FrcTrq = [J4CTorque J4RaialTorque J4Force J4RaialForce];
J4FrcTrq = num2cell(J4FrcTrq);
J4FrcTrq(1,:) = {'J4Mx (Nm)', 'J4My (Nm)', 'J4Mz (Nm)', 'J4RadialTorq (Nm)', 'J4Fx (N)', 'J4Fy (N)', 'J4Fz (N)', 'J4RadialForce (N)'};
writetable(cell2table(J4FrcTrq),'SimOutData\TrqFrc\J4TrqFrc.csv')

% J5 TrqFrc
J5FrcTrq = [J5CTorque J5RaialTorque J5Force J5RaialForce];
J5FrcTrq = num2cell(J5FrcTrq);
J5FrcTrq(1,:) = {'J5Mx (Nm)', 'J5My (Nm)', 'J5Mz (Nm)', 'J5RadialTorq (Nm)', 'J5Fx (N)', 'J5Fy (N)', 'J5Fz (N)', 'J5RadialForce (N)'};
writetable(cell2table(J5FrcTrq),'SimOutData\TrqFrc\J5TrqFrc.csv')

% J6 TrqFrc
J6FrcTrq = [J6CTorque J6RaialTorque J6Force J6RaialForce];
J6FrcTrq = num2cell(J6FrcTrq);
J6FrcTrq(1,:) = {'J6Mx (Nm)', 'J6My (Nm)', 'J6Mz (Nm)', 'J6RadialTorq (Nm)', 'J6Fx (N)', 'J6Fy (N)', 'J6Fz (N)', 'J6RadialForce (N)'};
writetable(cell2table(J6FrcTrq),'SimOutData\TrqFrc\J6TrqFrc.csv')

% All Axis Inertia Izz
AllInertia = [J1Izz; J2Izz; J3Izz; J4Izz; J5Izz; J6Izz]';
AllInertia = num2cell(AllInertia);
AllInertia(1,:) = {'J1 Izz (KgM^2)', 'J2 Izz (KgM^2)', 'J3 Izz (KgM^2)', 'J4 Izz (KgM^2)', 'J5 Izz (KgM^2)', 'J6 Izz (KgM^2)'};
writetable(cell2table(AllInertia),'SimOutData\Inertia\AllAxisInertia.csv')


% All Axis Joint Torque
AllTrq = [J1Torque J2Torque J3Torque J4Torque J5Torque J6Torque];
AllTrq = num2cell(AllTrq);
AllTrq(1,:) = {'J1 Trq (Nm)', 'J2 Trq (Nm)', 'J3 Trq (Nm)', 'J4 Trq (Nm)', 'J5 Trq (Nm)', 'J6 Trq (Nm)'};
writetable(cell2table(AllTrq),'SimOutData\JointTrq\AllAxisTrq.csv')


% End Effector Linear&Angular Velocity
EEVel = [EEXVel EEYVel EEZVel EEWxVel EEWyVel EEWzVel];
EEVel = num2cell(EEVel);
EEVel(1,:) = {'X-Axis (M/sn)', 'Y-Axis (M/sn)', 'Z-Axis (M/sn)', 'X-Axis (rad/sn)', 'Y-Axis (rad/sn)', 'Z-Axis (rad/sn)'};
writetable(cell2table(EEVel),'SimOutData\EndEffectorVelocity\EndEffectorVelocity.csv')


% All Axis Angle 
AllAngle = deg2rad(SimTheta);
AllAngle = num2cell(AllAngle);
AllAngle(1,:) = {'J1 (Rad)', 'J2 (Rad)', 'J3 (Rad)', 'J4 (Rad)', 'J5 (Rad)', 'J6 (Rad)'};
writetable(cell2table(AllAngle),'SimOutData\AllAxisAngleVelAcc\AllAngle.csv')


% All Axis Anglular Velocity 
AllAngularVel = deg2rad(SimDTheta);
AllAngularVel = num2cell(AllAngularVel);
AllAngularVel(1,:) = {'J1 (Rad/Sn)', 'J2 (Rad/Sn)', 'J3 (Rad/Sn)', 'J4 (Rad/Sn)', 'J5 (Rad/Sn)', 'J6 (Rad/Sn)'};
writetable(cell2table(AllAngularVel),'SimOutData\AllAxisAngleVelAcc\AllAngularVel.csv')

AllAngularVelrpm = SimDTheta./6;
AllAngularVelrpm = num2cell(AllAngularVelrpm);
AllAngularVelrpm(1,:) = {'J1 (Rpm)', 'J2 (Rpm)', 'J3 (Rpm)', 'J4 (Rpm)', 'J5 (Rpm)', 'J6 (Rpm)'};
writetable(cell2table(AllAngularVelrpm),'SimOutData\AllAxisAngleVelAcc\AllAngularVelrpm.csv')


% All Axis Anglular Acceleration
AllAngularAcc = deg2rad(SimDDTheta);
AllAngularAcc = num2cell(AllAngularAcc);
AllAngularAcc(1,:) = {'J1 (Rad/Sn^2)', 'J2 (Rad/Sn^2)', 'J3 (Rad/Sn^2)', 'J4 (Rad/Sn^2)', 'J5 (Rad/Sn^2)', 'J6 (Rad/Sn^2)'};
writetable(cell2table(AllAngularAcc),'SimOutData\AllAxisAngleVelAcc\AllAngularAcc.csv')

% EE Trajectory
EETrajData = [EEPoseSim EEOrientSim];
EETrajData = num2cell(EETrajData);
EETrajData(1,:) = {'EE X (Mm)', 'EE Y (Mm)', 'EE Z (Mm)', 'EE Roll (Deg)', 'EE Pitch (Deg)', 'EE Yaw (Deg)'};
writetable(cell2table(EETrajData),'SimOutData\EEPose\EETraj.csv')


% Simulation Time
SimTimeData = [tSimulink];
SimTimeData = num2cell(SimTimeData);
writetable(cell2table(SimTimeData),'SimOutData\EEPose\SimTime.csv')
