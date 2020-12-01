function [cost, step, success, trajectory] = CreateTrajectory(r, room, fis, maxStep, deltaT)
step = 0;
dist = 0;
success = true;
targetY = room(4)+room(6);
trajectory = [r];
vel = [0 0];
F = 0.5;
m = 1;
while r(2) <= targetY
    if step > maxStep
        success = false;
        break;
    end
    
    angle = evalfis(fis, [r(1) r(2) vel(1) vel(2)])*pi;
    vel = vel + deltaT*(F/m)*[cos(angle) sin(angle)];
    stepDist = vel*deltaT + 0.5*deltaT^2*(F/m);
    newr = r+stepDist;
    if hitWall(newr, room)
        success = false;
        break;
    end
    
    r = newr;
    trajectory = [trajectory;r];
    dist = dist+sqrt(stepDist(1)^2+stepDist(2)^2);
    step = step+1;
end

if success
    cost = dist;
else
    cost = 100*DistanceToTarget(r, [0 targetY+0.5]);
end

end

function dist = DistanceToTarget(r, target)
    dist = sqrt((r(1)-target(1))^2+(r(2)-target(2))^2);
end

function hit = hitWall(r, room)
hit = false;
if r(1) <= room(1) || r(1) >= room(2) || r(2) <= room(3)
    hit = true;
elseif (r(1) <= -0.5*room(5) || r(1) >= 0.5*room(5)) && (r(2)>= room(4) && r(2) <= room(4)+room(6))
    hit = true;
end
end