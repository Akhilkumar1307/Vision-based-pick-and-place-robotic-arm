function theta = ikine3r(l, conf, sigma)
% IKINE3R solves the inverse kinematics for a 3R planar robot
%
% [THETA1 THETA2 THETA3] = ikine3r([l1 l2 l3], [X Y PHI], SIGMA)
%
% Returns a vector of 3 angles corresponding to rotations about
% the three joints.
%
% sigma= +/-1 is a flag that gives us one of the two possible
% solutions
%
xx = conf(1)-l(3)*cos(conf(3));
yy = conf(2)-l(3)*sin(conf(3));
theta(1)= atan2(yy,xx)+...
sigma*acos((l(1)*l(1)+xx*xx+yy*yy-l(2)*l(2))/...
(2*l(1)*sqrt(xx*xx+yy*yy)));
if isreal(theta(1))==1
theta(2)=atan2(yy-l(1)*sin(theta(1)),...
xx-l(1)*cos(theta(1)))-theta(1);
if isreal(theta(2))==1
theta(3)=conf(3)-theta(1)-theta(2);
else
    disp('theta is complex')
end
end