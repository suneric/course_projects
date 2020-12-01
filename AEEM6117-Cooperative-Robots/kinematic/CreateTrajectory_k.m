% Create Trajectory and evaluate cost of trajectory
% for the cost
% we have two goals
% 1. move the rod out of the opening
% 2. make a shortest travel distance
% so have two parts of the cost, 
% 1: distance to the target place with more weight for fail (hit wall, or exceed max step)
% 2: total travel distance with less weight for success
function [cost, step, success, trajectory] = CreateTrajectory_k(sample, room, L, fisPos, fisOri, maxStep)
cost = 0;
r1 = sample(1:2);
r2 = sample(3:4);
rodAngle = sample(5);
targetY = room(4)+room(6);
trajectory = [r1 r2 false];
step = 0;
target = [0, targetY+L]; % predefine a target for evaluate the distance from robots
while r1(2) <= targetY || r2(2) <= targetY
    if step > maxStep
        success = false;
        distToTarget = DistanceToTarget(r1, target) + DistanceToTarget(r2,target); 
        cost = 100*distToTarget;
        break;
    end
    
    [travelDist, r1, r2, rodAngle, hit] = RobotsMove(r1, r2, rodAngle, fisPos, fisOri, L, room);
    trajectory = [trajectory; r1 r2 hit];
    
    if hit
        success = false;
        distToTarget = DistanceToTarget(r1, target) + DistanceToTarget(r2,target);
        cost = 100*distToTarget;
        break;
    end
    
    cost = cost+travelDist;
    step = step+1;
end
end

function dist = DistanceToTarget(r, target)
    dist = sqrt((r(1)-target(1))^2+(r(2)-target(2))^2);
end

