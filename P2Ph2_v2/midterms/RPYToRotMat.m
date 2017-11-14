function [rotMat] = RPYToRotMat(rpy)
%RPY to Rotation Matrix about ZXY.

phi = rpy(1);
theta = rpy(2);
psi = rpy(3);

rotMat = [ cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta); ...
    -cos(phi)*sin(psi), cos(phi)*cos(psi), sin(phi);...
    cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(theta)];


end

