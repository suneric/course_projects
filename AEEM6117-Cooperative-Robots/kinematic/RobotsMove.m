%
% robots move control
%
function [tDist, r1, r2, angle, hit] = RobotsMove(r1, r2, angle, fisPos, fisOri, L, room)
tDist = 0; % total distance travel
rate = 0.1; % displacement rate
index = randi(2); % give a probability to move either position first or orientation first
if index == 1
    [dist, r1, r2, hit] = MovePosition(r1, r2, angle, fisPos, room, rate);
    if hit
        return;
    end
    tDist = tDist + dist;
    
    [dist, r2, angle, hit] = MoveOrientation(r1, r2, angle, fisOri, L, room, rate);
    if hit
        return;
    end
    tDist = tDist + dist;
else
    [dist, r2, angle, hit] = MoveOrientation(r1, r2, angle, fisOri, L, room, rate);
    if hit
        return;
    end
    tDist = tDist + dist;
    
    [dist, r1, r2, hit] = MovePosition(r1, r2, angle, fisPos, room, rate);
    if hit
        return;
    end
    tDist = tDist + dist;
end
end

function [dist, r1, r2, hit] = MovePosition(r1, r2, angle, fisPos, room, rate)
    dist = 0;
    options = evalfisOptions('NumSamplePoints',101); % default 101
    dp = evalfis(fisPos, [r1(1) r1(2) angle/pi], options)*rate;
    newr1 = r1 + dp;
    newr2 = r2 + dp;
    hit = PositionCheck(newr1, newr2, room);
    if hit
        return;
    end
    
    dist = 2*sqrt(dp(1)^2 + dp(2)^2);
    r1 = newr1;
    r2 = newr2;
end

function [dist, r2, angle, hit] = MoveOrientation(r1, r2, angle, fisOri, L, room, rate)
    dist = 0;
    options = evalfisOptions('NumSamplePoints',101); % default 101
    do = evalfis(fisOri, [r2(1) r2(2) angle/pi], options)*pi*rate;
    newr2 = r1 + L*[cos(angle+do) sin(angle+do)];
    hit = PositionCheck(r1, newr2, room);
    if hit
        return;
    end
    
    dist = abs(do)*L;
    r2 = newr2;
    angle = AdjustAngle(angle+do);
end