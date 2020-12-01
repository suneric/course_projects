% calculate robot position with center of rod
function [r1, r2] = RobotPosition(rod, L)
    center = [rod(1) rod(2)];
    theta = rod(3);
    r1 = center - 0.5*L*[cos(theta) sin(theta)];
    r2 = center + 0.5*L*[cos(theta) sin(theta)];
end