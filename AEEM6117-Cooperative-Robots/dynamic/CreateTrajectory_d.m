% Create Trajectory and evaluate cost of trajectory
%
function [cost, step, success, trajectory] = CreateTrajectory_d(sample, room, M, L, fis, fis1, fis2, deltaT, maxStep)
trajectory = [];
cost = 0;
step = 0;
totalDist = 0;
r1 = sample(1:2);
r2 = sample(3:4);
rodPos = [0.5*(r1(1)+r2(1)) 0.5*(r1(2)+r2(2)) sample(5)];
rodVel = [0 0 0];
success = true;
targetY = room(4)+room(6);
target = [0 targetY+L];
while r1(2) <= targetY || r2(2) <= targetY
    % exceed max time
    if step > maxStep
        success = false;
        distToTarget = DistanceToTarget(r1, target) + DistanceToTarget(r2,target); 
        cost = 100*distToTarget;
        break;
    end
    
    % eval force and angle of rotation for robots
    [rodPos, rodVel, dist, hit] = RobotControl(fis, fis1, fis2, rodPos, rodVel, room, M, L, deltaT);
    [r1, r2] = RobotPosition(rodPos, L);
    trajectory = [trajectory; r1 r2 hit];
    if hit
        success = false;
        distToTarget = DistanceToTarget(r1, target) + DistanceToTarget(r2,target);
        cost = 100*distToTarget;
        break;
    end
    
    totalDist = totalDist + dist;
    step = step+1;
end

if success
    cost = totalDist;
end

%save('trajecotry.txt', '-append', 'trajectory', '-ascii');
end

function dist = DistanceToTarget(r, target)
    dist = sqrt((r(1)-target(1))^2+(r(2)-target(2))^2);
end
