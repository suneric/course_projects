% Adjust the angle to make it in range [-pi, pi]
%
function angle = AdjustAngle(angle)
if abs(angle) > 2*pi
    angle = mod(angle, 2*pi);
end

if angle >= pi
    angle = angle - 2*pi;
elseif angle < -pi
    angle = angle + 2*pi;
end
end