function trainingSamples = RandomRobotPosition(room, number)
    trainingSamples = zeros(number, 2);
    for i=1:number
        pos = rand(1, 2);
        r = [(room(2)-room(1))*pos(1)+room(1) (room(4)-room(3))*pos(2)+room(3)] ; % x in (-1, 1); y in (-1, 0)
        trainingSamples(i,:) = r;
    end
end