% -------------------- Robot Dynamic Parameters -------------------- % 
% For More Information -> "https://www.mathworks.com/help/sm/ug/specify-custom-inertia.html"

% -----------------------------------------
BaseMass = 92990.85/1e3;  % Base Mass [Kg]
BaseCoM  = [-0.0806806, 0.00066486, 0.0867692]; % Base Center Of Mass [mm] (x,y,z)
BaseMI   = [1.19097, 2.67515, 3.30952]; % Base Moment of Inertia [Kgm^2] (Ixx,Iyy,Izz)
BasePI   = [-0.000895122, 0.18626, 0.0145592]; % Base Product of Inertia [Kgm^2] (Iyz,Izx,Ixy)
% -----------------------------------------
Link1Mass = 171667.16/1e3;  % Link1 Mass [Kg]
Link1CoM  = [0.0599127, 0.0117726, 0.242027]; % Link1 Center Of Mass [m] (x,y,z)
Link1MI   = [5.22751, 6.57319, 4.01706]; % Link1 Moment of Inertia [Kgm^2] (Ixx,Iyy,Izz)
Link1PI   = [-0.0978596, -1.60283, 0.0626339]; % Link1 Product of Inertia [Kgm^2] (Iyz,Izx,Ixy)
% -----------------------------------------
Link2Mass = 61512.85/1e3;  % Link2 Mass [Kg]
Link2CoM  = [0.00268508, 0.401378, 0.119181]; % Link2 Center Of Mass [m] (x,y,z)
Link2MI   = [4.08737, 0.394247, 4.15979]; % Link2 Moment of Inertia [Kgm^2] (Ixx,Iyy,Izz)
Link2PI   = [-0.340813, 0.00362094, 0.0758729]; % Link2 Product of Inertia [Kgm^2] (Iyz,Izx,Ixy)
% -----------------------------------------
Link3Mass = 112250.52/1e3;  % Link3 Mass [Kg]
Link3CoM  = [0.0162415, -0.0305176, -0.206861]; % Link3 Center Of Mass [m] (x,y,z)
Link3MI   = [3.10027, 1.72104, 2.40403]; % Link3 Moment of Inertia [Kgm^2] (Ixx,Iyy,Izz)
Link3PI   = [0.197186, -0.0113439, 0.25102]; % Link3 Product of Inertia [Kgm^2] (Iyz,Izx,Ixy)
% -----------------------------------------
Link4Mass = 46373.56/1e3;  % Link4 Mass [Kg]
Link4CoM  = [-6.19234e-05, -0.0280677, 0.380145]; % Link4 Center Of Mass [m] (x,y,z)
Link4MI   = [2.28637, 2.22994, 0.213773]; % Link4 Moment of Inertia [Kgm^2] (Ixx,Iyy,Izz)
Link4PI   = [0.302667, 0.000614819, 7.43192e-05]; % Link4 Product of Inertia [Kgm^2] (Iyz,Izx,Ixy)
% -----------------------------------------
Link5Mass = 15836.95/1e3;  % Link5 Mass [Kg]
Link5CoM  = [-7.34754e-05, -0.0348301, 0.0553116]; % Link5 Center Of Mass [m] (x,y,z)
Link5MI   = [0.10221, 0.0607438, 0.0994571]; % Link5 Moment of Inertia [Kgm^2] (Ixx,Iyy,Izz)
Link5PI   = [-0.011554, -6.26731e-05, 0.000102105]; % Link5 Product of Inertia [Kgm^2] (Iyz,Izx,Ixy)
% -----------------------------------------
Link6Mass = 1110.79/1e3;  % Link6 Mass [Kg]
Link6CoM  = [3.94915e-05, -2.44371e-07, 0.0114446]; % Link6 Center Of Mass [m] (x,y,z)
Link6MI   = [0.00134579, 0.00134291, 0.00246432]; % Link6 Moment of Inertia [Kgm^2] (Ixx,Iyy,Izz)
Link6PI   = [-1.82558e-08, -7.08438e-07, -3.33693e-07]; % Link6 Product of Inertia [Kgm^2] (Iyz,Izx,Ixy)


