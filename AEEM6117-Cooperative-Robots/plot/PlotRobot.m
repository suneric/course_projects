% Plot robot in the figure
% input: figure f
% input: room
% input: center of rod [x y angle]
% input: length of rod L
% output: figure
function PlotRobot(room, robotData, L)
hold on

% plot room

roomx = linspace(room(1),room(2),100);
roomy = linspace(room(3),room(4)+1,100);
plot(roomx, 0, 'k');
plot(0, roomy, 'k');

title('Trajectory','FontSize', 20);
xlabel('X Position', 'FontSize',20);
ylabel('Y Position', 'FontSize', 20);
%legend({'ro = Robot1', 'b^ = Robot2'}, 'Location','northeast', 'FontSize', 20);

% plot door
plot([room(1) -room(5)/2], [room(4) room(4)], 'k');
plot([room(1) -room(5)/2], [room(4)+room(6) room(4)+room(6)], 'k');
plot([room(5)/2, room(2)], [room(4) room(4)], 'k');
plot([room(5)/2, room(2)], [room(4)+room(6) room(4)+room(6)], 'k');
%rectangle('Position',[0,room()])

plot([-room(5)/2, -room(5)/2], [room(4) room(4)+room(6)], 'k');
plot([room(5)/2, room(5)/2], [room(4) room(4)+room(6)], 'k');
plot([room(1) room(2)],[room(4)+1 room(4)+1], 'k');
plot([room(2) room(2)],[room(3) room(4)+1], 'k');

hit = robotData(5);
% plot robot
robot1 = robotData(1:2);
robot2 = robotData(3:4);
if hit
    plot([robot1(1) robot2(1)],[robot1(2) robot2(2)], 'r');
else
    plot([robot1(1) robot2(1)],[robot1(2) robot2(2)], 'k');
end
plot(robot1(1),robot1(2),'ro');
plot(robot2(1),robot2(2),'b^');

%hold off
end