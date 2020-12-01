% controller of robots
% input
function [pos, vel, dist, hit] = RobotControl(fis, fis1, fis2, pos, vel, room, M, L, deltaT)
[r1, r2] = RobotPosition(pos, L);
rodAngle = pos(3);
robotAng1 = evalfis([r1(1) r1(2)], fis); 
robotAng2 = evalfis([r2(1) r2(2)], fis);
forceAng1 = evalfis([robotAng1 rodAngle/pi], fis1)*pi; 
forceAng2 = evalfis([robotAng2 rodAngle/pi], fis2)*pi;
robotAngles(1) = AdjustAngle(forceAng1 - rodAngle);
robotAngles(2) = AdjustAngle(forceAng2 - rodAngle);
forces = [0.1 robotAngles(1) 0.1 robotAngles(2)];
[pos, vel, dist] = Simulation(forces, pos, vel, M, L, deltaT);
[r1, r2] = RobotPosition(pos, L);
hit = PositionCheck(r1, r2, room);
end

function [pos, vel, dist] = Simulation(forces, pos, vel, M, L, deltaT)
    angle = pos(3); % angle of rod
    f1 = forces(1); % force of robot1
    ra1 = forces(2); % angle between rod and robot1
    f2 = forces(3); % force of robot2
    ra2 = forces(4); % angle between rod and robot2
    % calculate acceleration of center of rod
    acc = [(f1*cos(ra1+angle)+f2*cos(ra2+angle))/M (f1*sin(ra1+angle)+f2*sin(ra2+angle))/M 6*(f1*sin(ra1)-f2*sin(ra2))/(M*L)];
    % simulate with Euler intergation
    vel = vel + deltaT*acc;
    dDist = deltaT*vel+0.5*deltaT^2*acc;
    pos = pos + dDist; 
    pos(3) = AdjustAngle(pos(3)); % make sure the angle is in range [-pi, pi]
    
    % calculate total distance travelled by two robots
    dist = 2*sqrt(dDist(1)^2+dDist(2)^2) + L*abs(dDist(3)); 
end