% get random training sample, position and orientation of center of the rod
% input: length of rod
% input: room [xmin xmax ymin ymax doorw doord]
% output: a list of [x y angle]
function trainingSamples = CreateRandomSamples(L, room, number)
    trainingSamples = zeros(number, 5);
    for i=1:number
        pos = rand(1, 2);
        r1 = [(room(2)-room(1))*pos(1)+room(1) (room(4)-room(3))*pos(2)+room(3)] ; % x in (-1, 1); y in (-1, 0)
        [angMin, angMax] = AngleRange(r1, room, L);
        angle = (angMax-angMin)*rand(1,1)+angMin;
        angle = AdjustAngle(angle);
        r2 = r1 + [L*cos(angle) L*sin(angle)];
        trainingSamples(i,:) = [r1 r2 angle];
    end
end

function [min, max] = AngleRange(pos, room, L)
x = pos(1);
y = pos(2);
dMinX = abs(x-room(1));
dMaxX = abs(x-room(2));
dMinY = abs(y-room(3));
dMaxY = abs(y-room(4));

if dMinX < L
    if dMinY < L
        min = -0.5*pi+acos(dMinY/L);
        max = pi-acos(dMinX/L);
    elseif dMaxY < L
        min = -pi+acos(dMinX/L);
        max = 0.5*pi-acos(dMaxY/L);
    else
        min = -pi + acos(dMinX/L);
        max = pi-acos(dMinX/L);
    end
elseif dMaxX < L
    if dMinY < L
        min = acos(dMaxX/L);
        max = 1.5*pi-acos(dMinY/L);
    elseif dMaxY < L
        min = 0.5*pi+acos(dMaxY/L);
        max = 2*pi-acos(dMaxX/L);
    else
        min = acos(dMaxX/L);
        max = 2*pi-acos(dMaxX/L);
    end
else
    if dMinY < L
        min = -0.5*pi+acos(dMinY/L);
        max = 1.5*pi-acos(dMinY/L);
    elseif dMaxY < L
        min = 0.5*pi+acos(dMaxY/L);
        max = 2.5*pi-acos(dMaxY/L);
    else
        min = -pi;
        max = pi;
    end
end
end
