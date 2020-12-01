function PlotSamples(samples, room, L, titlestr)
hold on
% plot room
roomx = linspace(room(1),room(2),100);
roomy = linspace(room(3),room(4)+1,100);
plot(roomx, 0, 'k');
plot(0, roomy, 'k');
% plot door
plot([room(1) -room(5)/2], [room(4) room(4)], 'k');
plot([room(1) -room(5)/2], [room(4)+room(6) room(4)+room(6)], 'k');
plot([room(5)/2, room(2)], [room(4) room(4)], 'k');
plot([room(5)/2, room(2)], [room(4)+room(6) room(4)+room(6)], 'k');
plot([-room(5)/2, -room(5)/2], [room(4) room(4)+room(6)], 'k');
plot([room(5)/2, room(5)/2], [room(4) room(4)+room(6)], 'k');
plot([room(1) room(2)],[room(4)+1 room(4)+1], 'k');
plot([room(2) room(2)],[room(3) room(4)+1],'k')

title(titlestr,'FontSize',20)

count = size(samples, 1);
for i=1:count
    robot1 = samples(i,1:2);
    robot2 = samples(i,3:4);
    plot([robot1(1) robot2(1)],[robot1(2) robot2(2)], 'k');
    plot(robot1(1),robot1(2),'ko');
    plot(robot2(1),robot2(2),'k^');
end
end