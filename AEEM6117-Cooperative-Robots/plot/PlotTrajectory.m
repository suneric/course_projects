%
function PlotTrajectory(trajectory, room, L, deltaT, clear)
figure('Name', 'Robot Trajectory');
subplot(1,1,1);
count = size(trajectory,1); % get row count
for i = 1:count
    if clear
        clf;
    end
    PlotRobot(room,trajectory(i,:),L);
    pause(deltaT);
end
plot(trajectory(:,1), trajectory(:,2), 'r')
plot(trajectory(:,3), trajectory(:,4), 'b')
end