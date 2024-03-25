function [X,Y,Z,Roll,Pitch,Yaw] = ForwardKinematics(Theta,DhParam)
% Joint and DH-Parameters
q1 = Theta(1);
q2 = Theta(2);
q3 = Theta(3);
q4 = Theta(4);
q5 = Theta(5);
q6 = Theta(6); 

a2 = DhParam(1,2);
a3 = DhParam(1,3);
a4 = DhParam(1,4);
d1 = DhParam(3,1);
d4 = DhParam(3,4);
d6 = DhParam(3,6);

% KR70 Forward Kinematic Matrix
T06 = [sind(q6)*(cosd(q4)*sind(q1)-sind(q4)*(cosd(q1)*cosd(q2)*cosd(q3)-cosd(q1)*sind(q2)*sind(q3)))+cosd(q6)*(cosd(q5)*(sind(q1)*sind(q4)+cosd(q4)*(cosd(q1)*cosd(q2)*cosd(q3)-cosd(q1)*sind(q2)*sind(q3)))-sind(q5)*(cosd(q1)*cosd(q2)*sind(q3)+cosd(q1)*cosd(q3)*sind(q2))), cosd(q6)*(cosd(q4)*sind(q1)-sind(q4)*(cosd(q1)*cosd(q2)*cosd(q3)-cosd(q1)*sind(q2)*sind(q3)))-sind(q6)*(cosd(q5)*(sind(q1)*sind(q4)+cosd(q4)*(cosd(q1)*cosd(q2)*cosd(q3)-cosd(q1)*sind(q2)*sind(q3)))-sind(q5)*(cosd(q1)*cosd(q2)*sind(q3)+cosd(q1)*cosd(q3)*sind(q2))), sind(q5)*(sind(q1)*sind(q4)+cosd(q4)*(cosd(q1)*cosd(q2)*cosd(q3)-cosd(q1)*sind(q2)*sind(q3)))+cosd(q5)*(cosd(q1)*cosd(q2)*sind(q3)+cosd(q1)*cosd(q3)*sind(q2)), a2*cosd(q1)+a4*cosd(q2+q3)*cosd(q1)+d4*sind(q2+q3)*cosd(q1)+a3*cosd(q1)*cosd(q2)+d6*sind(q2+q3)*cosd(q1)*cosd(q5)+d6*sind(q1)*sind(q4)*sind(q5)-d6*cosd(q1)*cosd(q4)*sind(q2)*sind(q3)*sind(q5)+d6*cosd(q1)*cosd(q2)*cosd(q3)*cosd(q4)*sind(q5);
     -sind(q6)*(cosd(q1)*cosd(q4)-sind(q4)*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1)))-cosd(q6)*(cosd(q5)*(cosd(q1)*sind(q4)+cosd(q4)*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1)))+sind(q5)*(cosd(q2)*sind(q1)*sind(q3)+cosd(q3)*sind(q1)*sind(q2))), sind(q6)*(cosd(q5)*(cosd(q1)*sind(q4)+cosd(q4)*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1)))+sind(q5)*(cosd(q2)*sind(q1)*sind(q3)+cosd(q3)*sind(q1)*sind(q2)))-cosd(q6)*(cosd(q1)*cosd(q4)-sind(q4)*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1))), cosd(q5)*(cosd(q2)*sind(q1)*sind(q3)+cosd(q3)*sind(q1)*sind(q2))-sind(q5)*(cosd(q1)*sind(q4)+cosd(q4)*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1))), d4*(cosd(q2)*sind(q1)*sind(q3)+cosd(q3)*sind(q1)*sind(q2))-a4*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1))+a2*sind(q1)-d6*(sind(q5)*(cosd(q1)*sind(q4)+cosd(q4)*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1)))-cosd(q5)*(cosd(q2)*sind(q1)*sind(q3)+cosd(q3)*sind(q1)*sind(q2)))+a3*cosd(q2)*sind(q1);
      cosd(q6)*(cosd(q2+q3)*sind(q5)+sind(q2+q3)*cosd(q4)*cosd(q5))-sind(q2+q3)*sind(q4)*sind(q6), -sind(q6)*(cosd(q2+q3)*sind(q5)+sind(q2+q3)*cosd(q4)*cosd(q5))-sind(q2+q3)*cosd(q6)*sind(q4), sind(q2+q3)*cosd(q4)*sind(q5)-cosd(q2+q3)*cosd(q5), d1-d4*cosd(q2+q3)+a4*sind(q2+q3)+a3*sind(q2)+(d6*sind(q2+q3)*sind(q4+q5))/2-d6*cosd(q2+q3)*cosd(q5)-(d6*sind(q4-q5)*sind(q2+q3))/2;
      0, 0, 0, 1];

X = T06(1,4);
Y = T06(2,4);
Z = T06(3,4);
TmpAngle = rad2deg(rotm2eul(T06(1:3,1:3),'XYZ')); % Be carefull rotation sequence
Roll  = TmpAngle(1);
Pitch = TmpAngle(2);
Yaw   = TmpAngle(3);
end