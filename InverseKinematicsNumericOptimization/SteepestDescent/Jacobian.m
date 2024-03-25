function [J] = Jacobian(Theta,DhParam)
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

% % Upper Jacobian Matrix Value (X-Y-Z)
Px = [d6*cosd(q1)*sind(q4)*sind(q5)-a4*cosd(q2+q3)*sind(q1)-d4*sind(q2+q3)*sind(q1)-a3*cosd(q2)*sind(q1)-d6*sind(q2+q3)*cosd(q5)*sind(q1)-a2*sind(q1)-d6*cosd(q2)*cosd(q3)*cosd(q4)*sind(q1)*sind(q5)+d6*cosd(q4)*sind(q1)*sind(q2)*sind(q3)*sind(q5), d4*cosd(q2+q3)*cosd(q1)-a4*sind(q2+q3)*cosd(q1)-a3*cosd(q1)*sind(q2)+d6*cosd(q2+q3)*cosd(q1)*cosd(q5)-d6*cosd(q1)*cosd(q2)*cosd(q4)*sind(q3)*sind(q5)-d6*cosd(q1)*cosd(q3)*cosd(q4)*sind(q2)*sind(q5), d4*cosd(q2+q3)*cosd(q1)-a4*sind(q2+q3)*cosd(q1)+d6*cosd(q2+q3)*cosd(q1)*cosd(q5)-d6*cosd(q1)*cosd(q2)*cosd(q4)*sind(q3)*sind(q5)-d6*cosd(q1)*cosd(q3)*cosd(q4)*sind(q2)*sind(q5), d6*cosd(q4)*sind(q1)*sind(q5)-d6*cosd(q1)*cosd(q2)*cosd(q3)*sind(q4)*sind(q5)+d6*cosd(q1)*sind(q2)*sind(q3)*sind(q4)*sind(q5), d6*cosd(q5)*sind(q1)*sind(q4)-d6*sind(q2+q3)*cosd(q1)*sind(q5)-d6*cosd(q1)*cosd(q4)*cosd(q5)*sind(q2)*sind(q3)+d6*cosd(q1)*cosd(q2)*cosd(q3)*cosd(q4)*cosd(q5), 0];
Py = [a2*cosd(q1)+d6*(sind(q5)*(sind(q1)*sind(q4)+cosd(q4)*(cosd(q1)*cosd(q2)*cosd(q3)-cosd(q1)*sind(q2)*sind(q3)))+cosd(q5)*(cosd(q1)*cosd(q2)*sind(q3)+cosd(q1)*cosd(q3)*sind(q2)))+a4*(cosd(q1)*cosd(q2)*cosd(q3)-cosd(q1)*sind(q2)*sind(q3))+d4*(cosd(q1)*cosd(q2)*sind(q3)+cosd(q1)*cosd(q3)*sind(q2))+a3*cosd(q1)*cosd(q2),-a4*(cosd(q2)*sind(q1)*sind(q3)+cosd(q3)*sind(q1)*sind(q2))-d4*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1))-d6*(cosd(q5)*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1))+cosd(q4)*sind(q5)*(cosd(q2)*sind(q1)*sind(q3)+cosd(q3)*sind(q1)*sind(q2)))-a3*sind(q1)*sind(q2),-a4*(cosd(q2)*sind(q1)*sind(q3)+cosd(q3)*sind(q1)*sind(q2))-d4*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1))-d6*(cosd(q5)*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1))+cosd(q4)*sind(q5)*(cosd(q2)*sind(q1)*sind(q3)+cosd(q3)*sind(q1)*sind(q2))), -d6*sind(q5)*(cosd(q1)*cosd(q4)-sind(q4)*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1))), -d6*(cosd(q5)*(cosd(q1)*sind(q4)+cosd(q4)*(sind(q1)*sind(q2)*sind(q3)-cosd(q2)*cosd(q3)*sind(q1)))+sind(q5)*(cosd(q2)*sind(q1)*sind(q3)+cosd(q3)*sind(q1)*sind(q2))), 0];
Pz = [0, a4*cosd(q2+q3)+d4*sind(q2+q3)+a3*cosd(q2)+(d6*cosd(q2+q3)*sind(q4+q5))/2+d6*sind(q2+q3)*cosd(q5)-(d6*sind(q4-q5)*cosd(q2+q3))/2, a4*cosd(q2+q3)+d4*sind(q2+q3)+(d6*cosd(q2+q3)*sind(q4+q5))/2+d6*sind(q2+q3)*cosd(q5)-(d6*sind(q4-q5)*cosd(q2+q3))/2, (d6*cosd(q4+q5)*sind(q2+q3))/2-(d6*cosd(q4-q5)*sind(q2+q3))/2, (d6*cosd(q4+q5)*sind(q2+q3))/2+d6*cosd(q2+q3)*sind(q5)+(d6*cosd(q4-q5)*sind(q2+q3))/2, 0];
% Lower Jacobian Matrix Value (Roll-Pitch-Yaw)
Rx = [0, sind(q1), sind(q1), sind(q2+q3)*cosd(q1), cosd(q4)*sind(q1)-cosd(q1)*cosd(q2)*cosd(q3)*sind(q4)+cosd(q1)*sind(q2)*sind(q3)*sind(q4), sind(q1)*sind(q4)*sind(q5)+sind(q2+q3)*cosd(q1)*cosd(q5)+cosd(q1)*cosd(q2)*cosd(q3)*cosd(q4)*sind(q5)-cosd(q1)*cosd(q4)*sind(q2)*sind(q3)*sind(q5)];
Ry = [0, -cosd(q1), -cosd(q1), sind(q2+q3)*sind(q1), sind(q1)*sind(q2)*sind(q3)*sind(q4)-cosd(q2)*cosd(q3)*sind(q1)*sind(q4)-cosd(q1)*cosd(q4), sind(q2+q3)*cosd(q5)*sind(q1)-cosd(q1)*sind(q4)*sind(q5)+cosd(q2)*cosd(q3)*cosd(q4)*sind(q1)*sind(q5)-cosd(q4)*sind(q1)*sind(q2)*sind(q3)*sind(q5)];
Rz = [1, 0, 0, -cosd(q2+q3), -sind(q2+q3)*sind(q4), sind(q2+q3)*cosd(q4)*sind(q5)-cosd(q2+q3)*cosd(q5)];
J = [Px; Py; Pz; Rx; Ry; Rz];

end