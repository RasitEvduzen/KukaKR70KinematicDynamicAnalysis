function [Output] = DHMatrix(alpha,a,d,theta)

Rtheta = (pi/180)*theta;    % Deg to Rad
Ralpha = (pi/180)*alpha;    % Deg to Rad


Output = [ cos(Rtheta), -sin(Rtheta)*cos(Ralpha),  sin(Rtheta)*sin(Ralpha),  a*cos(Rtheta),
           sin(Rtheta),  cos(Rtheta)*cos(Ralpha), -cos(Rtheta)*sin(Ralpha),  a*sin(Rtheta),
                     0,              sin(Ralpha),              cos(Ralpha),              d,
                     0,                        0,                        0,              1];


end

