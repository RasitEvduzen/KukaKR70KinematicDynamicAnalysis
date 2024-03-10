clc,clear all,close all,warning off;
%% 6 DOF ROBOTIC ARM FORWARD & INVERSE KINEMATIC ANALYSIS
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

%------------------------Welding Torch Pose-----------------------%
WTorchX = -6.56;  % mm
WTorchY = 8.01;
WTorchZ = 431.33; 
%-----------------------------------------------------------------%
%-----------------Cartesian Space Trajectory----------------------%
% Cartesian Trajectory- 1

% cx = (0:pi:300*pi)+500;
% cy = 500*sin(2e-2*cx)-250;
% cz = 500*cos(2e-2*cx)+1500;
% Nod = 100;
% lx1 = linspace(cx(1,end),cx(1,end),Nod); 
% ly1 = linspace(cy(1,end),-250,Nod);
% lz1 = linspace(cz(1,end),1500,Nod);
% 
% lx2 = linspace(cx(1,end),-750,Nod); 
% ly2 = linspace(-250,-250,Nod);
% lz2 = linspace(1500,1500,Nod);
% 
% cx = [cx lx1 lx2];
% cy = [cy ly1 ly2];
% cz = [cz lz1 lz2];

%-----------------------------------------------------------------%
% Cartesian Trajectory- 2
% cx = linspace(500,500,50);
% cy = linspace(0,500,50); 
% cz = linspace(1000,1000,50); 

%-----------------------------------------------------------------%
% Cartesian Park Pose
% cx = linspace(1395,1395,2);
% cy = linspace(0,0,2);
% cz = linspace(1515,1515,2);

%-----------------------------------------------------------------%
% Cartesian Trajectory- 3
% Nod = 100;
% lx1 = linspace(-1000,1000,Nod);
% ly1 = linspace(1000,1750,Nod);
% lz1 = linspace(1000,1000,Nod);
% 
% lx2 = linspace(1000,-1000,Nod);
% ly2 = linspace(1750,750,Nod);
% lz2 = linspace(1000,2000,Nod);
% 
% lx3 = linspace(-1000,-1000,Nod);
% ly3 = linspace(750,750,Nod);
% lz3 = linspace(2000,1500,Nod);
% 
% lx4 = linspace(-1000,1000,Nod);
% ly4 = linspace(750,750,Nod);
% lz4 = linspace(1500,1500,Nod);
% 
% lx5 = linspace(1000,-1000,Nod);
% ly5 = linspace(750,1000,Nod);
% lz5 = linspace(1500,1000,Nod);
% 
% cx = [lx1 lx2 lx3 lx4 lx5];
% cy = [ly1 ly2 ly3 ly4 ly5];
% cz = [lz1 lz2 lz3 lz4 lz5];
% cx(1,end+1) = cx(1,end);
% cy(1,end+1) = cy(1,end);
% cz(1,end+1) = cz(1,end);
%-----------------------------------------------------------------%
% Cartesian Trajectory- 4
width = 550;
height = 550;
depth = 550;
NoD = 100;
xini = 1000;
yini = 0;
zini = 1250;


x1 = linspace(xini,xini,NoD);
y1 = linspace(yini,yini,NoD); 
z1 = linspace(zini,zini+height,NoD); 

x2 = linspace(xini,xini,NoD);
y2 = linspace(yini,yini+width,NoD); 
z2 = linspace(zini+height,zini+height,NoD); 

x3 = linspace(xini,xini,NoD);
y3 = linspace(yini+width,yini+width,NoD); 
z3 = linspace(zini+height,zini,NoD); 

x4 = linspace(xini,xini,NoD);
y4 = linspace(yini+width,yini,NoD); 
z4 = linspace(zini,zini,NoD); 

x5 = linspace(xini,xini-depth,NoD);
y5 = linspace(yini,yini,NoD); 
z5 = linspace(zini,zini,NoD); 

x6 = linspace(xini-depth,xini-depth,NoD);
y6 = linspace(yini,yini,NoD); 
z6 = linspace(zini,zini+height,NoD); 

x7 = linspace(xini-depth,xini,NoD);
y7 = linspace(yini,yini,NoD); 
z7 = linspace(zini+height,zini+height,NoD); 

x8 = linspace(xini,xini,NoD);
y8 = linspace(yini,yini+width,NoD); 
z8 = linspace(zini+height,zini+height,NoD); 

x9 = linspace(xini,xini-depth,NoD);
y9 = linspace(yini+width,yini+width,NoD); 
z9 = linspace(zini+height,zini+height,NoD); 

x10 = linspace(xini-depth,xini-depth,NoD);
y10 = linspace(yini+width,yini+width,NoD); 
z10 = linspace(zini+height,zini,NoD); 

x11 = linspace(xini-depth,xini,NoD);
y11 = linspace(yini+width,yini+width,NoD); 
z11 = linspace(zini,zini,NoD); 

x12 = linspace(xini,xini-depth,NoD);
y12 = linspace(yini+width,yini+width,NoD); 
z12 = linspace(zini,zini,NoD);

x13 = linspace(xini-depth,xini-depth,NoD);
y13 = linspace(yini+width,yini,NoD); 
z13 = linspace(zini,zini,NoD);

x14 = linspace(xini-depth,xini-depth,NoD);
y14 = linspace(yini,yini,NoD); 
z14 = linspace(zini,zini+height,NoD);

x15 = linspace(xini-depth,xini-depth,NoD);
y15 = linspace(yini,yini+width,NoD); 
z15 = linspace(zini+height,zini+height,NoD);

cx = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13  x14 x15 x15(1,end)];
cy = [y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11 y12 y13  y14 y15 y15(1,end)];
cz = [z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 z13  z14 z15 z15(1,end)];
%-----------------------------------------------------------------%

% Cartesian Trajectory- 5
% NoD = 100;
% cx = linspace(500,1000,NoD);
% cy = linspace(0,500,NoD); 
% cz = linspace(1000,1500,NoD); 
%-----------------------------------------------------------------%

% Cartesian Trajectory- 6
% NoD = 100;
% Ax1 = linspace(500,1000,NoD);
% Ay1 = linspace(500,500,NoD); 
% Az1 = linspace(1500,1500,NoD);
% Ax2 = linspace(1000,1000,NoD);
% Ay2 = linspace(500,0,NoD); 
% Az2 = linspace(1500,1500,NoD); 
% Ax3 = linspace(1000,500,NoD);
% Ay3 = linspace(0,0,NoD); 
% Az3 = linspace(1500,1500,NoD);
% Ax4 = linspace(750,750,NoD);
% Ay4 = linspace(0,500,NoD); 
% Az4 = linspace(1500,1500,NoD);
% cx = [Ax1 Ax2 Ax3 Ax4]; 
% cy = [Ay1 Ay2 Ay3 Ay4];
% cz = [Az1 Az2 Az3 Az4];
%-----------------------------------------------------------------%



ToolPose = [];
AllTargetPose = eye(4);
AllMatrix = [];
eeOrient = [0 0 1; 0 -1 0; 1 0 0 ];


for pq = 1:size(cx,2)
    AllTargetPose(:,:,pq) = [eeOrient [cx(1,pq) cy(1,pq) cz(1,pq)]' ; 0 0 0 1];
end

%-----------------------------------------------------------------%

InverseAngle = [];
%------------------ Calculate Inverse Kinematics -----------------%
for qp=1:size(cx,2)-1

iversethetaInitial = IK(AllTargetPose(:,:,qp),DhParam);
iversethetaFinal   = IK(AllTargetPose(:,:,qp+1),DhParam);

%-------------------- Trajectory Generation ----------------------%

qini   = iversethetaInitial;
qini(find(abs(qini)<1e-10)) = 0;
qfinal = iversethetaFinal;
qfinal(find(abs(qfinal)<1e-10)) = 0;

InverseAngle = [InverseAngle; qfinal];

time = 1e-2;       % Sampling Periode
SimTheta = [];

for j=1:length(qini)
    [q] = LinearTrajectoryGeneration(qini(j), qfinal(j), time,0,1);
    SimTheta   = [SimTheta q];
end

%--------- ROBOT Forward Kinematics Calculate Matrix -----------%
for k=1:size(SimTheta,1)
    T06 = eye(4,4);
    for i=1:size(SimTheta,2)
        temp = T06;
        T(:,:,i) = DHMatrixModify(alpha(i),a(i),d(i),SimTheta(k,i));
        T06 = T06 * T(:,:,i);
        AllMatrix = [AllMatrix; T06];
    end
    ToolPose = [ToolPose T06(1:3,4)];
end
end

tSim = size(InverseAngle,1) * time;
tSimulink = linspace(0,tSim,size(InverseAngle,1))';
RobotBaseMotion = linspace(-500,0,size(InverseAngle,1))'*1e-3;


ToolPose = ToolPose';
ToolPose(:,1) = ToolPose(:,1) + WTorchZ;
ToolPose(:,2) = ToolPose(:,2) + WTorchY;
ToolPose(:,3) = ToolPose(:,3) - WTorchX;
ToolPose(find(abs(ToolPose)<1e-10)) = 0;
EEPose = ToolPose;
ToolPose = unique(ToolPose,'rows');
EESim = ToolPose;


% return
%% Robot Simulation
set(gcf,'Position',[1 1 1720 900],'Color','white')


% gif('Traj3.gif')

for qp = 1:24:size(AllMatrix,1)

    clf
    trplot(eye(4),'frame','0','thick',1,'rgb','length',300), hold on, grid on
    plot3(cx,cy,cz,'k','LineWidth',.3)
    axis equal,axis([-2500 2500 -2500 2500 0 2500])
    view(45,20)  % First arguman is azimuth angle, second is elevation angle
    TransPlot = AllMatrix(qp:qp+23,:);
    k = 1;
    TempPlot1 = eye(4);
    for i=1:6
        
        TempPlot2 = TransPlot(k:i*4,:);
        plot3([TempPlot1(1,4) TempPlot2(1,4)],[TempPlot1(2,4) TempPlot2(2,4)],[TempPlot1(3,4) TempPlot2(3,4)],'k','LineWidth',1);
        TempPlot1 = TempPlot2;
        xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('KR70 Robot Inverse Kinematics Simulation')
        trplot(TempPlot2,'frame',num2str(i),'thick',1,'rgb','length',300)
        k = k + 4;
    end
    scatter3(TempPlot2(1,4),TempPlot2(2,4),TempPlot2(3,4),'filled','r')
    pause(eps)
%     gif
end


return
%% Plot EE Trajectory
figure
set(gcf,'Position',[1 1 1720 900])
plot3(ToolPose(:,1),ToolPose(:,2),ToolPose(:,3),'r','LineWidth',5),hold on,grid on
plot3(cx(1,1:end-1),cy(1,1:end-1),cz(1,1:end-1),'k--','LineWidth',2)
xlabel('X-Axis (mm)'),ylabel('Y-Axis (mm)'),zlabel('Z-Axis (mm)')
view(140,20)  % First arguman is azimuth angle, second is elevation angle
title('End Effector Trajectory')
legend('Robot EE Trajectory','Referance Trajectory')


