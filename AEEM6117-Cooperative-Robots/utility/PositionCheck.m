%--------g   h---------
%        6   7
%        |   |
%d---4---e   f----5---c
%|                    |
%1                    3 
%|         *          |
%a---------2----------b
function hit = PositionCheck(r1, r2, room)
hit = false;
a = [room(1) room(3)];
b = [room(2) room(3)];
c = [room(2) room(4)];
d = [room(1) room(4)];
e = [-0.5*room(5) room(4)];
f = [0.5*room(5) room(4)];
g = [-0.5*room(5) room(4)+room(6)];
h = [0.5*room(5) room(4)+room(6)];

if hitWall(r1, room) || hitWall(r2, room)
    hit = true;
    return;
end

if intersectionCheck(r1, r2, a, d)... 
    || intersectionCheck(r1, r2, a, b)...
    || intersectionCheck(r1, r2, b, c)...
    || intersectionCheck(r1, r2, d, e)...
    || intersectionCheck(r1, r2, f, c)...
    || intersectionCheck(r1, r2, g, e)...
    || intersectionCheck(r1, r2, h, f)
    hit = true;
end

end

function hit = hitWall(r, room)
hit = false;
if r(1) <= room(1) || r(1) >= room(2) || r(2) <= room(3)
    hit = true;
elseif (r(1) <= -0.5*room(5) || r(1) >= 0.5*room(5)) && (r(2)>= room(4) && r(2) <= room(4)+room(6))
    hit = true;
end
end

% check if line segment a,b has intersection with line segment c, d
function intersected = intersectionCheck(a, b, c, d)
intersected = true;
area_abc = (a(1)-c(1))*(b(2)-c(2))-(a(2)-c(2))*(b(1)-c(1));
area_abd = (a(1)-d(1))*(b(2)-d(2))-(a(2)-d(2))*(b(1)-d(1));
if area_abc*area_abd > 0
    intersected = false;
    return;
end
area_cda = (c(1)-a(1))*(d(2)-a(2))-(c(2)-a(2))*(d(1)-a(1));
area_cdb = area_cda + area_abc - area_abd;
if area_cda*area_cdb > 0 
    intersected = false;
    return;
end
end