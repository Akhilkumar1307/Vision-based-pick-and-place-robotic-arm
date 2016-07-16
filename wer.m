clc
a = ikine3r([9 8 17],[10 0 -pi/4],1);
d = toDegrees('radians',a)
s = d + [0 90 90]
if ((min(s)<0)||(max(s)>180))
    disp('Not possible');
end
view(0,90);
ThreeLink.plot(a);

