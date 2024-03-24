function [Output] = DHMatrixModify(alpha,a,d,theta)

Rtheta = (pi/180)*theta;    % Deg to Rad
Ralpha = (pi/180)*alpha;    % Deg to Rad


Output = [            cos(Rtheta),            -sin(Rtheta),            0,              a
          sin(Rtheta)*cos(Ralpha), cos(Rtheta)*cos(Ralpha), -sin(Ralpha), -sin(Ralpha)*d
          sin(Rtheta)*sin(Ralpha), cos(Rtheta)*sin(Ralpha),  cos(Ralpha),  cos(Ralpha)*d
                                0,                       0,           0,               1];


end

