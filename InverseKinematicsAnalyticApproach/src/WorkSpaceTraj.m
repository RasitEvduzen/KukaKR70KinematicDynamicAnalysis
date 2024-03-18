function [Traj] = WorkSpaceTraj(TrajSelect,NoD)

switch TrajSelect
    case 0
        % Home Pose
        NoD = 2;
        cx = linspace(1395,1395,NoD)';
        cy = linspace(0,0,NoD)';
        cz = linspace(1515,1515,NoD)';
        Traj = [cx cy cz];
    case 1
        % Cartesian Trajectory- 1
        cx = (0:2*pi:200*pi)+500;
        cy = 500*sin(2e-2*cx)-250;
        cz = 500*cos(2e-2*cx)+1500;
        lx1 = linspace(cx(1,end),cx(1,end),NoD);
        ly1 = linspace(cy(1,end),-250,NoD);
        lz1 = linspace(cz(1,end),1500,NoD);
        lx2 = linspace(cx(1,end),500,NoD);
        ly2 = linspace(-250,-250,NoD);
        lz2 = linspace(1500,1500,NoD);
        cx = [cx lx1 lx2]';
        cy = [cy ly1 ly2]';
        cz = [cz lz1 lz2]';
        Traj = [cx cy cz];
    case 2
        % Cartesian Trajectory- 2
        cx = linspace(500,1000,NoD)';
        cy = linspace(0,500,NoD)';
        cz = linspace(1000,1500,NoD)';
        Traj = [cx cy cz];
    case 3
        width = 550;
        height = 550;
        depth = 550;
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

        cx = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 x14 x15 x15(1,end)]';
        cy = [y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11 y12 y13 y14 y15 y15(1,end)]';
        cz = [z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 z13 z14 z15 z15(1,end)]';
        Traj = [cx cy cz];
    case 4
        t1x = linspace(1500,1500,NoD);
        t1y = linspace(-500,0,NoD); 
        t1z = linspace(1000,2000,NoD); 

        t2x = linspace(1500,1500,NoD);
        t2y = linspace(-0,500,NoD); 
        t2z = linspace(2000,1000,NoD); 

        t3x = linspace(1500,1500,NoD);
        t3y = linspace(500,-555,NoD); 
        t3z = linspace(1000,1670.43,NoD); 

        t4x = linspace(1500,1500,NoD);
        t4y = linspace(-555,555,NoD); 
        t4z = linspace(1670.43,1670.43,NoD); 

        t5x = linspace(1500,1500,NoD);
        t5y = linspace(555,-500,NoD); 
        t5z = linspace(1670.43,1000,NoD); 

        cx = [t1x t2x t3x t4x t5x]';
        cy = [t1y t2y t3y t4y t5y]';
        cz = [t1z t2z t3z t4z t5z]';
        Traj = [cx cy cz];
    case 5
        t = linspace(0,6.1*pi,NoD);
        x = linspace(1000,1000,size(t,2))';
        y = (700*sin(t/3)+500)';
        z = (-700*sin(t/3).*cos(t/3)+1000)';
        Traj = [x y z];
    otherwise
        disp('Select allowable trajectory!')
end
end